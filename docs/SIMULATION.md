# Standalone bench demo (Stage 2)

`pio run -e esp32dev_sim` builds the firmware with `-DW17_SIM_LINK2_FEEDER`, which injects a
scripted link2 drive through the **real** assembler + monitor path — so the sound and lights
run with no board #1 attached. Flash it, connect the MAX98357A + speaker and the WS2812
strip, and open the serial monitor (115200) to watch the phase narration.

## 21 s demo loop

| t (s) | phase | sound | lights |
|---|---|---|---|
| 0–2 | IDLE | starter crank → idle | crank comet sweep, fire-up flash → halo teal (armed) |
| 2–6 | DRIVING | revs sweep, gears climb | DRS tell: rear-bar edge pixels green above ~70% throttle |
| 6–8 | ERS_DEPLOY | ERS whine layer over the engine | — |
| 8–9.5 | BRAKE_HARVEST | engine drops, overrun crackle | brake bar + rain light flashing (harvest) |
| 9.5–11 | CORNERING | part-throttle | indicators sweep L/R |
| 11–12 | DROPOUT | **engine to silence** | **amber hazard blink** (staleness → local failsafe) |
| 12–14 | RECOVERED | engine returns | back to normal |
| 14–15.5 | PARKED | engine winds down to silence | dim-white halo (disarmed; the contrast beat before the show) |
| 15.5–19.5 | SHOWCASE | re-crank, then `ShowScript`'s seeded gentle idle blips over the burble | halo: slow teal breathe with the tail lit (link2 `modeFlags` bit0) |
| 19.5–21 | SHOW_LOWBATT | engine falls silent | slow red halo pulse, **no hazard** (the frame stream stays alive and healthy) |

The DROPOUT phase is the important one: the feeder emits **nothing** for 1 s, so the
`Link2Monitor` crosses its 500 ms staleness threshold, projects the per-field failsafe state
(engine silent, hazard lights), and then recovers cleanly on the next frame.

PARKED, SHOWCASE and SHOW_LOWBATT are designed, not defects, so a bench operator should not
file them as faults: only the 11–12 s DROPOUT is a fault (stale link ⇒ hazard). SHOWCASE
sends the truthful `armed=0` / `throttlePercent=0` frame that a real showcase boot of board
#1 puts on the wire (`modeFlags` bit0 set) — board #2 supplies the whole presentation
locally (ignition keys on `armed || showcase`; `lib/enginesim`'s `ShowScript` adds the
blips). SHOW_LOWBATT's silent engine plus a red pulse with no hazard is the deliberate
"asking for the charger" ending: the link is healthy the whole time, so nothing is faulted.

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
- [ ] Brightness cap (`LightConfig::maxBrightness` = 110) is applied **before** gamma, so it is
      ~43% *perceptual* but only ~16% *electrical* — a full channel is driven at 40/255 duty, and
      every quiet state lives below that. Judge the dim states (disarmed halo, red tail, showcase
      breathe floor — all set to `kMinVisibleDuty`) in daylight and retune if they vanish; raise
      the cap only after checking the UBEC headroom.
- [ ] **Trim the TX before judging the indicators.** Today's truth (correctness-1's other half,
      NOT fixed on this branch — nothing self-cancels a trimmed-off-centre stick): the indicator
      logic only self-cancels once `|steer| < indicatorOffPercent` (20). A physical steering trim
      that leaves the centred stick reporting `steeringPercent` anywhere in `[20, 40)` (below
      `indicatorOnPercent` = 40, so it never re-latches, but at or above `indicatorOffPercent`, so
      an already-latched indicator never releases) makes that indicator blink forever with the
      wheel centred. Set the TX trim so the centred stick reads below `indicatorOffPercent` before
      judging the indicator behaviour on the bench — `[bench-TBD]`.
