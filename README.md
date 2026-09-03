# w17-soundlight-fw

Firmware for **ESP32 #2 "sound + light"** of the 1/10 FPV Mercedes W17 RC car — the companion
to [w17-control-fw](https://github.com/beforethenexttolast/w17-control-fw) (ESP32 #1).

It consumes the one-way **link2** UART stream from board #1 and drives:
- **Engine sound** — procedural synthesis (harmonic partial stack + noise + ERS whine) over
  I2S → MAX98357A → 4Ω speaker, with named voice profiles (`SynthProfiles.hpp`: **V10** is
  the boot default; the **V6 turbo-hybrid** voice is selected at runtime by the link2 v2
  `soundProfile` byte, and the v2 `volume` byte scales the output 0–100 with 0 = true
  silence — both persisted on board #1 only; unknown profile values fall back to the V10).
  A PCM sample player can drop in later behind the same `ISampleSource` seam (the "hybrid"
  choice).
- **WS2812 lights** — brake, turn indicators, halo, ignition-on animation (crank comet +
  fire-up flash), DRS-open tell, F1 rain light (flashes while ERS is *harvesting*),
  low-battery pulse, a showcase halo breathe, and a failsafe hazard blink that overrides
  everything (a link that never delivers a frame escalates to hazard after a short grace
  window).
- **Showcase mode** (link2 v2 `modeFlags` bit0) — board #1's stationary-demo boot sends a
  truthful `armed=0` / `throttlePercent=0` frame with the bit set; ignition authority on
  board #2 keys on `armed || showcase`, so the engine re-cranks and `ShowScript`
  (in `enginesim`) layers gentle seeded idle blips over it (the rev limiter and overrun
  crackle are unreachable by construction), while the halo runs a slow teal breathe with
  the tail lit. Failsafe, low-battery and a stale link all mute it — there is no local
  trigger on this board.

See `CLAUDE.md` for the architecture brief and `docs/link2_protocol.md` for the input frame
(copied from the control repo, which owns the protocol). `docs/SIMULATION.md` explains the
standalone bench demo.

## Build & test

```
pio test -e native        # host unit tests (no hardware)
pio run  -e esp32dev       # real firmware
pio run  -e esp32dev_sim   # standalone bench demo (scripted drive, no board #1)
```

## Module map

| lib | role | pure? |
|---|---|---|
| `link2` | frame codec + assembler (copied verbatim from the control repo) | yes |
| `link2monitor` | staleness watchdog, per-field failsafe projection, LinkStatus | yes |
| `enginesim` | virtual engine: rpm inertia, shift blips, ignition authority (`armed \|\| showcase`), rev limiter, overrun, `ShowScript`'s showcase idle envelope | yes |
| `audiodecision` | volume/profile composition, dead-man staleness gating for the audio task | yes |
| `audiostartup` | I2S device bring-up sequencing + cleanup policy | yes |
| `soundsynth` | `ISampleSource` + `EngineSynth` wavetable DSP (deterministic, integer); named voice profiles | yes |
| `lights` | compositor: brake/indicators/ignition/DRS/rain/halo/showcase-breathe/hazard, gamma, power budget | yes |
| `config` | header-only pin map for the sound/light board | yes |
| `audio_hal_esp32` | IDF legacy I2S output (stereo-duplicated mono) | esp32-only |
| `lights_hal_esp32` | WS2812 via Adafruit NeoPixel; the pure renderer's `Rgb[30]` output array is the seam (no `ILedStrip` interface exists) | esp32-only |

Dual-core: control loop on core 1, audio pump on core 0, one `std::atomic<uint32_t>` param
word + a heartbeat between them (see the cross-core rule in `CLAUDE.md`).
