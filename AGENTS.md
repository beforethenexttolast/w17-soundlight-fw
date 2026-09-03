# ESP32 #2 "Sound + Light Board" — Firmware Brief

> **Codex-session guidance. Corrected 2026-08-17 (the 2026-08-11 port inverted ownership) and
> 2026-09-03 (this file no longer copies CLAUDE.md's body — the copy had already drifted
> stale twice, on the voice-selection decision and the lights/ignition list). Source of
> truth: this repo's CLAUDE.md + the workspace CLAUDE.md/AGENTS.md.**

Companion firmware to `w17-control-fw` (ESP32 #1) for the 1/10 FPV Mercedes W17 RC car,
consuming the one-way **link2** UART stream from board #1 and turning it into engine sound +
WS2812 lights. Architecture, the module map, the pin map, and build/test commands: **read
CLAUDE.md** — this file deliberately carries no separate copy of any of it.

## Ownership and session role (read first)

This repo is **Claude-Code-owned** (registry: `WORKSPACE_MAP.md` at the workspace root). A
Codex session here is a **guest**: read-only by default; edits only when the owner's task
explicitly names this repo. All safety boundaries, gates, and invariants in CLAUDE.md bind
Codex sessions identically.

## No control authority (non-negotiable)

This board has **no vehicle control authority**. It must never command steering, ESC/throttle,
DRS, the gimbal, CRSF, or any vehicle motion. It only **consumes already-arbitrated state**
from board #1 over the one-way `link2` stream and turns it into sound + light. The `link2`
protocol is owned by `w17-control-fw`; this repo may consume and validate its local copy but
must **not** fork or casually redefine it — protocol changes happen there first.
