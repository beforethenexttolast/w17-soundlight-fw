#include <unity.h>

#include "enginesim/EngineSim.hpp"
#include "enginesim/ShowScript.hpp"

using enginesim::applyShowScript;
using enginesim::EngineSim;
using enginesim::Ignition;
using enginesim::showScriptThrottleAt;
using link2::VehicleState;

namespace {

VehicleState showcaseIdle() {
    VehicleState s;
    s.showcase = true;
    s.armed = false;
    s.failsafe = false;
    s.throttlePercent = 0; // the truthful wire value in showcase
    return s;
}

} // namespace

void setUp() {}
void tearDown() {}

// Deterministic and pure: the same instant always yields the same throttle
// -- across calls, across scan order, with no instance state anywhere. Every
// boot therefore plays the identical curated show and the tests below
// replay it exactly (the soundsynth seeded-LFSR house rule).
void test_deterministic_pure_function_of_time() {
    for (uint32_t t = 0; t <= 120000; t += 7) {
        TEST_ASSERT_EQUAL_UINT8(showScriptThrottleAt(t), showScriptThrottleAt(t));
    }
    // Spot values sampled out of order match an in-order scan.
    const uint32_t probes[] = {60011, 3, 99999, 42000, 3, 60011};
    uint8_t first[6];
    for (int i = 0; i < 6; ++i) first[i] = showScriptThrottleAt(probes[i]);
    TEST_ASSERT_EQUAL_UINT8(first[1], first[4]); // t=3 twice
    TEST_ASSERT_EQUAL_UINT8(first[0], first[5]); // t=60011 twice
}

// The D8 envelope, scanned over five minutes of show at 10 ms resolution:
// never above the 30 % ceiling, mostly at 0 (idle -- the engine's own
// wobble carries those stretches), but with real blips that actually reach
// gentle-rev territory.
void test_envelope_bounded_mostly_idle_with_real_blips() {
    uint32_t samples = 0;
    uint32_t nonzero = 0;
    uint8_t maxSeen = 0;
    for (uint32_t t = 0; t <= 300000; t += 10) {
        const uint8_t v = showScriptThrottleAt(t);
        ++samples;
        if (v > 0) ++nonzero;
        if (v > maxSeen) maxSeen = v;
        TEST_ASSERT_TRUE(v <= enginesim::kShowMaxBlipPct); // ceiling, every sample
    }
    // Mostly 0: blips occupy 3/8 slots for 400-700 ms of a 4000 ms slot
    // (expected duty ~5 %); assert an order-of-magnitude envelope so a
    // re-curated seed cannot silently turn the idle into a rave.
    TEST_ASSERT_TRUE(nonzero * 100 < samples * 25); // < 25 % duty
    TEST_ASSERT_TRUE(nonzero > 0);                  // the show does blip
    TEST_ASSERT_TRUE(maxSeen >= enginesim::kShowMinBlipPct); // peaks are real
}

// Gentle by shape, not just by ceiling: the triangle's worst slope is
// peak/(duration/2) <= 30/200 per ms, so one 20 ms control tick can never
// step the throttle by more than 3 points -- an order of magnitude under
// the overrun detector's 40-point cliff (which the static_asserts already
// rule out on the ceiling alone).
void test_blip_slope_is_gentle_at_control_tick_rate() {
    uint8_t prev = showScriptThrottleAt(0);
    for (uint32_t t = 20; t <= 300000; t += 20) {
        const uint8_t v = showScriptThrottleAt(t);
        const int step = (v > prev) ? (v - prev) : (prev - v);
        TEST_ASSERT_TRUE(step <= 4);
        prev = v;
    }
}

// applyShowScript gating: the script touches the throttle ONLY under
// showcase sound authority. Drive frames, failsafe frames, low-battery
// frames and armed+showcase liar frames all pass through untouched -- the
// drive path cannot be contaminated by construction.
void test_apply_show_script_gating() {
    // Find a time inside a blip so "active" is distinguishable from 0.
    uint32_t blipT = 0;
    for (uint32_t t = 0; t <= 300000; t += 10) {
        if (showScriptThrottleAt(t) > 0) {
            blipT = t;
            break;
        }
    }
    TEST_ASSERT_TRUE(blipT > 0);

    // Active: showcase idle frame -> script throttle substituted, every
    // other field untouched.
    VehicleState in = showcaseIdle();
    in.steeringPercent = -40;
    in.gear = 2;
    in.volume = 55;
    VehicleState out = applyShowScript(in, blipT);
    TEST_ASSERT_EQUAL_INT8(static_cast<int8_t>(showScriptThrottleAt(blipT)),
                           out.throttlePercent);
    TEST_ASSERT_TRUE(out.throttlePercent > 0);
    TEST_ASSERT_EQUAL_INT8(-40, out.steeringPercent);
    TEST_ASSERT_EQUAL_UINT8(2, out.gear);
    TEST_ASSERT_EQUAL_UINT8(55, out.volume);
    TEST_ASSERT_TRUE(out.showcase);

    // Passthrough: a DRIVING frame keeps its real throttle even at blip time.
    VehicleState drive;
    drive.armed = true;
    drive.failsafe = false;
    drive.throttlePercent = 80;
    TEST_ASSERT_EQUAL_INT8(80, applyShowScript(drive, blipT).throttlePercent);

    // Passthrough: armed outranks showcase (liar frame).
    drive.showcase = true;
    TEST_ASSERT_EQUAL_INT8(80, applyShowScript(drive, blipT).throttlePercent);

    // Passthrough: showcase + failsafe (D4 row 3) stays at the wire's 0.
    VehicleState fs = showcaseIdle();
    fs.failsafe = true;
    TEST_ASSERT_EQUAL_INT8(0, applyShowScript(fs, blipT).throttlePercent);

    // Passthrough: showcase + lowBattery (D5: the show ends, script stops).
    VehicleState low = showcaseIdle();
    low.lowBattery = true;
    TEST_ASSERT_EQUAL_INT8(0, applyShowScript(low, blipT).throttlePercent);

    // Passthrough: not showcase at all.
    VehicleState plain;
    plain.failsafe = false;
    TEST_ASSERT_EQUAL_INT8(0, applyShowScript(plain, blipT).throttlePercent);
}

// The composed guarantee (D8's "unreachable by construction", witnessed at
// runtime on top of the static_asserts): script -> EngineSim over 400 s of
// show. The limiter and the overrun crackle never trigger, the engine
// never leaves the gentle band, the ignition never drops out mid-show, and
// the engine's throttle output never exceeds the ceiling.
void test_composed_show_never_reaches_limiter_or_overrun() {
    EngineSim e;
    uint32_t t = 0;
    bool running = false;
    for (int i = 0; i < 20000; ++i) { // 400 s at the 20 ms control tick
        t += 20;
        e.update(t, applyShowScript(showcaseIdle(), t));
        const enginesim::EngineState& st = e.engine();
        TEST_ASSERT_FALSE(st.limiterActive);
        TEST_ASSERT_FALSE(st.overrunActive);
        TEST_ASSERT_TRUE(st.throttlePercent <= enginesim::kShowMaxBlipPct);
        if (running) {
            // Once caught, the show holds Running for its entire healthy run.
            TEST_ASSERT_EQUAL(Ignition::Running, st.ignition);
            // Gentle band: max blip target is idle + 30 % of the span (6950)
            // plus wobble/rounding -- far below the 60 % overrun band (10400)
            // and the limiter window (14750+).
            TEST_ASSERT_TRUE(st.engineRpm <= 7500);
        }
        running |= (st.ignition == Ignition::Running);
    }
    TEST_ASSERT_TRUE(running);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_deterministic_pure_function_of_time);
    RUN_TEST(test_envelope_bounded_mostly_idle_with_real_blips);
    RUN_TEST(test_blip_slope_is_gentle_at_control_tick_rate);
    RUN_TEST(test_apply_show_script_gating);
    RUN_TEST(test_composed_show_never_reaches_limiter_or_overrun);
    return UNITY_END();
}
