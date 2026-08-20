#pragma once

#include <cstdint>

#include "enginesim/EngineSim.hpp"
#include "link2/Link2Frame.hpp"

// The curated SHOWCASE idle script (showcase design D8, owner-accepted
// 2026-08-17): while board #1 authorizes the stationary demo over link2
// (modeFlags bit0), the engine idles on its existing wobble and this script
// adds an occasional GENTLE blip -- never a screaming rev, never a fake
// drive. It supplies the ONE input the truthful wire deliberately carries
// as 0 in showcase: a local throttle value for the virtual engine. Nothing
// else is scripted -- no gears, no ERS, no lights, no actuators (board #2
// has no actuators, and board #1's showcase non-goals forbid scripted
// motion outright).
//
// Deterministic and seeded (the soundsynth house rule): the envelope is a
// PURE FUNCTION of absolute time -- per-slot xorshift32 streams seeded from
// kShowScriptSeed and the slot index -- so every boot plays the same
// curated show, tests replay it exactly, and an interruption (failsafe,
// low battery) simply mutes it while the schedule keeps its phase.
//
// The rev limiter and the overrun crackle are UNREACHABLE BY CONSTRUCTION,
// not by tuning -- static_asserts below pin the envelope's ceiling against
// the EngineSimConfig defaults on all three independent grounds:
//   1. limiter needs throttle >= limiterThrottlePct; the script tops out at 30.
//   2. overrun needs a >= 40-point drop in ONE tick; a signal bounded by 30
//      cannot drop by 40 from any value it can reach, margins irrelevant.
//   3. overrun also needs rpm >= the 60 % band; a 30 % throttle target
//      (idle + 30 % of the span) never enters it.
namespace enginesim {

// Curated-envelope constants. Changing kShowScriptSeed re-curates the whole
// show (different blip times/heights); everything else bounds its shape.
inline constexpr uint32_t kShowScriptSeed = 0x0057F117u; // nonzero; "57 F1 17"
inline constexpr uint32_t kShowSlotMs = 4000;   // one scheduling slot
inline constexpr uint32_t kShowBlipSlotsIn8 = 3; // 3-in-8 slots carry a blip
inline constexpr uint8_t kShowMinBlipPct = 12;  // gentlest blip peak
inline constexpr uint8_t kShowMaxBlipPct = 30;  // D8 ceiling: <= 30 %
inline constexpr uint32_t kShowBlipMinMs = 400; // shortest blip
inline constexpr uint32_t kShowBlipMaxMs = 700; // longest blip
inline constexpr uint32_t kShowBlipMarginMs = 500; // silence pad at slot edges

// Structural unreachability of the limiter/overrun, pinned at compile time
// against the shipped EngineSimConfig defaults (main.cpp constructs the
// EngineSim with exactly these; a config change that broke a bound would
// stop this translation unit compiling, not soften the show at the bench).
static_assert(kShowMaxBlipPct < EngineSimConfig{}.limiterThrottlePct,
              "show blip ceiling must stay below the limiter's full-throttle gate");
static_assert(kShowMaxBlipPct < EngineSimConfig{}.overrunThrottleDrop,
              "a signal bounded by the blip ceiling can never produce the overrun drop");
static_assert(kShowMaxBlipPct < EngineSimConfig{}.overrunHighRpmPct,
              "the blip ceiling's rpm target must sit below the overrun high-rpm band");
static_assert(kShowMinBlipPct > 0 && kShowMinBlipPct <= kShowMaxBlipPct,
              "blip peak range inverted");
static_assert(kShowBlipMinMs >= 2 && kShowBlipMinMs <= kShowBlipMaxMs,
              "blip duration range inverted (and the triangle divides by duration/2)");
static_assert(kShowBlipMaxMs + 2 * kShowBlipMarginMs < kShowSlotMs,
              "a blip plus both margins must fit inside one slot");
static_assert(kShowScriptSeed != 0, "xorshift streams need a nonzero seed");

// The curated throttle envelope at an absolute time: 0 almost always (the
// engine's own idle wobble carries those stretches), a 0->peak->0 integer
// triangle of 12..30 % for 400..700 ms in 3 of 8 slots. Pure function --
// same input, same output, no state anywhere.
uint8_t showScriptThrottleAt(uint32_t nowMs);

// The single composition point src/main.cpp, the esp32dev_sim demo and the
// tests all share (the audiodecision pattern: one production decision, no
// per-caller copies): feed the engine the effective (post-staleness) state,
// with the script's throttle substituted ONLY under showcase sound
// authority. Everywhere else -- driving, disarmed, failsafe, low battery,
// stale link -- the frame passes through untouched, so the drive path
// cannot be contaminated by construction (and the wire throttle in showcase
// is truthfully 0, so substitution never masks a real command).
inline link2::VehicleState applyShowScript(link2::VehicleState s, uint32_t nowMs) {
    if (showcaseSoundAuthority(s)) {
        s.throttlePercent = static_cast<int8_t>(showScriptThrottleAt(nowMs));
    }
    return s;
}

} // namespace enginesim
