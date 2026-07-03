# Standalone bench demo (Stage 2)

`pio run -e esp32dev_sim` builds the firmware with `-DW17_SIM_LINK2_FEEDER`, which injects a
scripted link2 drive through the **real** assembler + monitor path — so the sound and lights
run with no board #1 attached. Flash it, connect the MAX98357A + speaker and the WS2812
strip, and open the serial monitor (115200) to watch the phase narration.

## ~14 s demo loop

| t (s) | phase | sound | lights |
|---|---|---|---|
| 0–2 | IDLE | starter crank → idle | halo teal (armed) |
| 2–6 | DRIVING | revs sweep, gears climb | — |
| 6–8 | ERS_DEPLOY | ERS whine layer over the engine | — |
| 8–9.5 | BRAKE_HARVEST | engine drops, overrun crackle | brake bar + rain light flashing (harvest) |
| 9.5–11 | CORNERING | part-throttle | indicators sweep L/R |
| 11–12 | DROPOUT | **engine to silence** | **amber hazard blink** (staleness → local failsafe) |
| 12–14 | RECOVERED | engine returns | back to normal |

The DROPOUT phase is the important one: the feeder emits **nothing** for 1 s, so the
`Link2Monitor` crosses its 500 ms staleness threshold, projects the per-field failsafe state
(engine silent, hazard lights), and then recovers cleanly on the next frame.

## First-run bench checklist (low-confidence platform facts)

- [ ] I2S produces clean audio at 22050 Hz through the legacy IDF driver on the pinned
      espressif32/core version (if silent/garbled, verify `channel_format` and that mono is
      transmitted as duplicated stereo — see `Esp32I2sAudio`).
- [ ] MAX98357A **GAIN** strap: start floating (9 dB); **SD_MODE** high. Drive SD_MODE from a
      GPIO with a post-clock delay only if the power-on pop is objectionable.
- [ ] WS2812: 330 Ω series on data, 1000 µF across strip 5 V/GND, 1N5819 on strip VDD
      (build-sheet fixes). Watch for LED glitches while audio DMA runs; if seen, move the
      `strip.show()` timing or switch to raw double-buffered RMT.
- [ ] Confirm the synth reads as "engine" on the actual speaker in the shell — if too thin,
      the V10 flavor / partial weighting / rpm range are all config knobs in
      `EngineSynthConfig`. If synthesis disappoints, the `ISampleSource` seam accepts a PCM
      sample renderer with no changes above it.
- [ ] Brightness cap (`LightConfig::maxBrightness`, ~43%) keeps 30 LEDs within the current
      budget even on the all-amber hazard; raise only after checking the UBEC headroom.
