# link2 protocol v1 — control board (ESP32 #1) → sound/light board (ESP32 #2)

One-way UART. **115200 baud, 8N1, 3.3 V logic, common ground.** Sender TX = ESP32 #1
GPIO25 → receiver RX on ESP32 #2. (GPIO26 is reserved for a future ack channel and is
not driven — do not connect anything to it yet.) Both boards power from the same UBEC
rail, so they come up together; avoid driving the line into an unpowered board #2 for
long periods.

Reference implementation: `lib/link2/` in this repo — **liftable wholesale** into the
board-#2 project (no dependencies beyond a byte-sink interface; the decoder and
`Link2FrameAssembler` are what board #2 needs).

## Ownership — and why this file is a copy

**`w17-control-fw` owns this protocol.** Every change lands there first; that repo's
`docs/link2_protocol.md` together with its `lib/link2/` are the definition. This board is
only the **receiver**, and what lives here are copies: `lib/link2/` is a verbatim copy of the
shared subset (`include/link2/Link2Frame.hpp`, `include/link2/Link2Codec.hpp`,
`src/Link2Codec.cpp`, `library.json`), deliberately omitting `src/Link2Sender.cpp`, which is
control-side only. **This document is a copy too.**

The copy is **permanent by decision (2026-07-25)**, not a bootstrap toward a submodule — a
submodule would drag the sender onto board #2 as dead code that PlatformIO's LDF would
compile, for a four-file library that has never drifted.

Because these are copies, they are **guarded**. The checker lives in the owner repo
(`tools/link2_copy_check.sh` there); there is deliberately **no copy of it here**, since a
drift checker kept in two places can drift from itself. This repo owns the **enforcement**:
the `link2-drift` job in `.github/workflows/ci.yml` checks out `w17-control-fw` and runs that
script in `--strict` mode, where an absent sibling is a hard failure rather than a quiet skip.
Two tiers, on purpose:

- **Fatal** — the four shared **code** files. They are compiled on both boards, so
  byte-identity is the right invariant and any difference is a bug. CI also distinguishes
  *drifted* from *could not check*: a guard that never ran has proven nothing, and must not
  read green.
- **Reported (a CI warning, not a failure)** — *this document*. Its normative content — field
  table, lengths, CRC, the 500 ms staleness rule — must not drift, but byte-identity is the
  wrong bar, because each copy legitimately carries repo-local prose such as the section you
  are reading. A diff cannot separate normative drift from local commentary, so CI surfaces
  the difference and leaves the judgement to a human.

Independently of the copy check, `pio test -e native` pins the wire format hermetically from
*this* side: `test_golden_frame_bytes` fixes the exact 14 bytes, and the assembler tests pin
the hard-reject and resync behaviour. (The CRC is additionally pinned against `lib/crsf` on
the owner side; this board has no `lib/crsf`.) Both layers are needed — a copy check cannot
tell you the format is *correct*, only that the two copies agree.

Amending this file alone changes nothing on the wire. **Protocol changes happen in
`w17-control-fw` first**, then the shared subset is re-copied here.

## Frame layout (14 bytes)

```
offset  size  field
0       1     start byte, always 0xA5
1       1     length = payload byte count (11 in v1)
2       11    payload (below)
13      1     crc8 over bytes [1..12]  (length + payload; start byte excluded)
```

CRC8: polynomial 0xD5, initial value 0, MSB-first, no reflection (CRC-8/DVB-S2 — the
same algorithm CRSF uses; catalog check value for ASCII `"123456789"` is `0xBC`).

**Validation order:** start → length → CRC → version. `BadVersion` therefore means a
*well-formed frame from a newer sender*, not corruption. Receivers must hard-reject an
unsupported length byte the moment it arrives (do not buffer `length` unknown bytes —
a corrupted 0xFF length would otherwise swallow ~1 s of following frames).

## Payload v1 — all multi-byte fields little-endian (low byte first)

| offset | size | field | semantics |
|---|---|---|---|
| 0 | 1 | version | `1`. Reject anything else. |
| 1 | 1 | throttlePercent | int8, −100…+100. **What the ESC is actually commanded** (0 while disarmed or failsafe) — engine sound should track this, not stick position. Negative = braking, **never reverse motion** (the ESC runs forward/brake). |
| 2 | 1 | steeringPercent | int8, −100…+100. Left/right for turn indicators. Live even while disarmed. |
| 3 | 1 | flags | bit0 braking (already hysteresis-filtered by the sender — drive the brake light from it directly), bit1 reverse (**reserved, always 0 in v1 — do not key anything off it**), bit2 drsOpen, bit3 armed, bit4 failsafe, bit5 lowBattery, bit6 ersDeploying (boost/overtake actively draining — e.g. an ERS whine sound layer), bit7 reserved (sender writes 0, **receivers must mask, never reject**). |
| 4 | 1 | gear | 1-based display gear, 1…4 (matches the firmware gearbox numGears). |
| 5–6 | 2 | rpm | uint16. **Wheel/axle rpm** (one magnet), plausible max ~5000 — *not* engine rpm; derive engine revs from throttlePercent or scale this. |
| 7–8 | 2 | batteryMv | uint16, 2S pack millivolts. Display garnish — the `lowBattery` flag is the authoritative judgment (calibrated, 3 s-qualified, hysteresis-latched on board #1). |
| 9 | 1 | ersPercent | 0…100, ERS energy store. Frozen (not zero) outside ERS mode. |
| 10 | 1 | driveMode | 0 = TRAINING, 1 = RACE (gearbox), 2 = ERS (gearbox + ERS deploy). Receivers may vary engine character per mode; treat unknown values as 1 (RACE). |

### driveMode: wire values vs display labels (audit R19, decided 2026-07-25)

`driveMode` is a **number on the wire**; the names above are the shipping *display* labels.
**TRAINING / RACE / ERS** is what a person reads — the ground-station HUD shows exactly
these. Two nearby spellings are deliberate, not drift:

- The iPhone canonical contract maps the same three values to the enum strings
  `TRAINING` / `GEARBOX` / `GEARBOX_ERS` (`shared/telemetrySnapshot.js` in the
  `w17-ground-station` repo). Those are **wire identifiers for a machine consumer**, not
  labels; the HUD maps them back to RACE / ERS before anything is displayed.
- Board #1 also puts `driveMode` in its CRSF FLIGHTMODE string as a bare integer
  (`"G%u M%u E%u"` → `M2`), so a handset showing the raw string displays **no mode label at
  all** and cannot diverge. This board emits no CRSF whatsoever — see the no-control-authority
  rule in `CLAUDE.md`.

So there is no user-visible contradiction to fix, and none of those surfaces needs to change.
For this board the practical rule is the one in the table: key engine character off the number,
and treat an unknown value as RACE.

## State matrix

| condition | throttlePercent | braking | armed | failsafe | telemetry (rpm/battery) |
|---|---|---|---|---|---|
| failsafe (link lost) | 0 | 0 | 0 | 1 | still live |
| disarmed idle | 0 | 0 | 0 | 0 | live |
| driving | as commanded | as filtered | 1 | 0 | live |

## Timing — and the one rule the receiver MUST implement

Frames are sent at a nominal **20 Hz** (every 50 ms). Receivers must tolerate jitter and
must not phase-lock to the rate.

**Mandatory staleness timeout:** if no CRC-valid frame arrives for **500 ms** (10 missed
frames), the receiver must enter its own local failsafe — engine sound to idle/off,
hazard blink. On a one-way link, a cut wire is otherwise indistinguishable from "the
last state persists forever".

## Worked example

The byte-identical frame is pinned by `test/test_link2/test_main.cpp`
(`test_golden_frame_bytes`):

```
A5 0B 01 2A E7 4C 03 DC 05 DC 1E 3C 02 CE
│  │  │  │  │  │  │  └─┴─ rpm 1500    └─┴─ battery 7900  │  │  └ crc8
│  │  │  │  │  │  └ gear 3                               │  └ driveMode 2 (ERS)
│  │  │  │  │  └ flags 0x4C = drsOpen | armed | ersDeploying
│  │  │  │  └ steeringPercent = 0xE7 = −25                └ ersPercent 60
│  │  │  └ throttlePercent = 0x2A = +42
│  │  └ version 1
│  └ length 11
└ start
```
Decoded: throttle +42 %, steering −25 %, DRS open, armed, ERS deploying at 60 %
store in ERS mode, no failsafe, gear 3, wheel 1500 rpm, battery 7.900 V.
