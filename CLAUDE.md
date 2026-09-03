# ESP32 #2 "Sound + Light Board" — Firmware Brief

Companion firmware to `w17-control-fw` (ESP32 #1) for the 1/10 FPV Mercedes W17 RC car.
This board consumes the one-way **link2** UART stream from board #1 and produces:
- **Engine sound** via I2S → MAX98357A → 4Ω 3W speaker (procedural V10-flavored synthesis;
  a PCM sample player can drop in later behind the same `ISampleSource` seam).
- **WS2812 lights** (30-LED strip): brake, turn indicators, halo, ignition-on animation,
  DRS-open tell, F1 rain light (flashes while ERS is *harvesting*), low-battery pulse,
  failsafe hazard, showcase halo breathe (link2 modeFlags bit0).

Input protocol: `docs/link2_protocol.md` (copied from the control repo, which owns it).
Receiver obligations from that doc are MANDATORY here: hard-reject unsupported length
bytes immediately, and **no CRC-valid frame for 500 ms ⇒ local failsafe** (engine to
silence, hazard blink).

## No control authority (non-negotiable)
This board has **no vehicle control authority**. It must never command steering, ESC/throttle,
DRS, the gimbal, CRSF, or any vehicle motion. It only **consumes already-arbitrated state**
from board #1 over the one-way `link2` stream and turns it into sound + light.

**link2 ownership:** the `link2` protocol is owned by `w17-control-fw`. This repo may
consume and validate its local copy but must **not** fork or casually redefine the protocol;
any protocol change must be coordinated with `w17-control-fw` (protocol changes happen there
first, then the codec is copied here verbatim — see the module map).

## Pin map (this board's own choices — bench-verify)
See `lib/config/include/config/PinMap.hpp`. UART RX from board #1 = GPIO16 (TX GPIO17
reserved for the future ack channel); I2S BCLK=26 / LRC=25 / DIN=22 (canonical MAX98357A
hookup); WS2812 data = GPIO4 via 330Ω. MAX98357A GAIN/SD_MODE strapping notes live in the
pin header.

## Architecture rules (same house style as w17-control-fw)
- Pure logic libs under `lib/` with **no Arduino headers**; thin `*_hal_esp32` impls
  referenced only from `src/main.cpp`; Unity tests in `test/` run on `[env:native]`.
- Config structs with `constexpr valid()` + `static_assert` at the definition site.
- Integer math in all control/render paths (float allowed only in one-time table setup).
- **Cross-core rule:** the ONLY surface shared between the core-1 control loop and the
  core-0 audio task is the packed `std::atomic<uint32_t>` synth-param word plus the
  heartbeat atomic. Synth phase state is audio-task-only; VehicleState / enginesim /
  lights are core-1-only. Do not reach across.
- The audio task carries a dead-man: params not refreshed for ~500 ms ⇒ volume ramps to 0
  (a wedged control loop must not leave the engine screaming).

## Module map
- `lib/link2` — frame codec + assembler, copied VERBATIM from w17-control-fw (do not
  fork; protocol changes happen there first). `encodeFrame` is kept for the sim feeder.
- `lib/link2monitor` — staleness watchdog + per-field effective state + LinkStatus
  (NeverConnected / Up / Lost).
- `lib/audiodecision` — pure audio-task decisions shared verbatim by `src/main.cpp` and the
  native tests: `synthVolumeFor` (ignition → base volume), `normalizeSoundProfile` (link2 v2
  `soundProfile` byte, reserved values fold to V10), `applyOperatorVolume` (composes the
  link2 v2 `volume` byte), and the audio-heartbeat dead-man boundary. Also `classifyWrite` /
  `runtimeActionFor` (SLR-4): any I2S short-write or driver error **permanently disables
  audio for the rest of the boot, no retry** — the only non-`Continue` action is terminal.
- `lib/audiostartup` — pure I2S startup sequencing (install → pins → DMA clear → task) with
  best-effort cleanup on a post-install failure; the real ESP-IDF calls live behind an `Ops`
  adapter.
- `lib/config` — header-only `PinMap.hpp`: this board's own GPIO assignments, independent of
  board #1's map.
- `lib/enginesim` — virtual engine: rpm inertia, gear-shift blips, ignition state
  machine (Off/Cranking/Running; keys on `armed || showcase`, where `showcase` is the link2
  v2 `modeFlags` bit0 — the armed path is the original behavior), rev limiter, overrun
  crackle window. Also home of `ShowScript`: the
  showcase idle script, a pure function of absolute time (deterministic; limiter/overrun
  unreachable by static_assert against the shipped config) — it generates throttle shapes
  ONLY under the showcase authority predicate and is NOT a local demo trigger: with no
  valid link2 frames the 500 ms staleness mandate still silences and hazards.
- `lib/soundsynth` — `ISampleSource` seam + `EngineSynth`: wavetable partial stack at the
  firing frequency (default 5 firings/rev = V10 flavor, range 3500–15000 rpm — chosen so
  the fundamental sits in a small speaker's band), per-rev AM, throttle-correlated noise,
  pitch-tracking ERS whine, param smoothing. Deterministic (seeded LFSR noise). Named
  voice profiles in `SynthProfiles.hpp` (V10 = boot default = wire 0, V6 turbo-hybrid = wire
  1), selected at runtime by the link2 v2 `soundProfile` byte (vision decision 15,
  2026-08-16): `audiodecision::normalizeSoundProfile` folds any reserved value to V10 on the
  control core, two bits of the packed synth word carry it across cores, and
  `EngineSynth::setVoiceProfile` applies it on the audio task. Operator volume (link2 v2
  `volume` byte, 0..100, 0 = true silence) composes the same way via
  `audiodecision::applyOperatorVolume` (`stateVolume * op / 100`, then the synth's own
  `sample * vol / 255` gain stage); failsafe/staleness always wins (`Ignition::Off` drives
  stateVolume to 0). No NVS or build flag on this board — board #1 persists both
  (`sound.profile` / `sound.volume`).
- `lib/lights` — pure compositor: base (halo incl. ignition-on animation and the showcase
  teal breathe with lit tail, distinct from the never-connected grace breathe) → DRS tell →
  brake/indicators/rain → low-battery → failsafe hazard override; never-connected shows a
  calm breathe for a bounded grace window, then escalates to hazard; gamma LUT; brightness
  cap with a static power budget in `valid()` (halo-capable colors are static_assert-pinned
  to the two-primary worst case that budget models).
- `lib/audio_hal_esp32` / `lib/lights_hal_esp32` — legacy IDF i2s driver (stereo-duplicated
  mono) and Adafruit NeoPixel behind `ILedStrip`.

## Build/test
`pio test -e native` (all suites incl. a pure end-to-end frames→audio test),
`pio run -e esp32dev`, `pio run -e esp32dev_sim` (standalone bench demo).
