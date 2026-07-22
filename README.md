# w17-soundlight-fw

Firmware for **ESP32 #2 "sound + light"** of the 1/10 FPV Mercedes W17 RC car — the companion
to [w17-control-fw](https://github.com/beforethenexttolast/w17-control-fw) (ESP32 #1).

It consumes the one-way **link2** UART stream from board #1 and drives:
- **Engine sound** — procedural V10-flavored synthesis (harmonic partial stack + noise + ERS
  whine) over I2S → MAX98357A → 4Ω speaker. A PCM sample player can drop in later behind the
  same `ISampleSource` seam (the "hybrid" choice).
- **WS2812 lights** — brake, turn indicators, halo, F1 rain light (flashes while ERS is
  *harvesting*), low-battery pulse, and a failsafe hazard blink that overrides everything.

See `CLAUDE.md` for the architecture brief and `docs/link2_protocol.md` for the input frame
(copied from the control repo, which owns the protocol). `docs/SIMULATION.md` explains the
standalone bench demo.

## Build & test

```
pio test -e native        # 94 host unit tests (no hardware)
pio run  -e esp32dev       # real firmware
pio run  -e esp32dev_sim   # standalone bench demo (scripted drive, no board #1)
```

## Module map

| lib | role | pure? |
|---|---|---|
| `link2` | frame codec + assembler (copied verbatim from the control repo) | yes |
| `link2monitor` | staleness watchdog, per-field failsafe projection, LinkStatus | yes |
| `enginesim` | virtual engine: rpm inertia, shift blips, ignition states, rev limiter, overrun | yes |
| `soundsynth` | `ISampleSource` + `EngineSynth` wavetable DSP (deterministic, integer) | yes |
| `lights` | compositor: brake/indicators/rain/halo/hazard, gamma, power budget | yes |
| `audio_hal_esp32` | IDF legacy I2S output (stereo-duplicated mono) | esp32-only |
| `lights_hal_esp32` | WS2812 via Adafruit NeoPixel behind `ILedStrip` | esp32-only |

Dual-core: control loop on core 1, audio pump on core 0, one `std::atomic<uint32_t>` param
word + a heartbeat between them (see the cross-core rule in `CLAUDE.md`).
