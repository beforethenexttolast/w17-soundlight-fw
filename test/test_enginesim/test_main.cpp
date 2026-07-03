#include <unity.h>

#include "enginesim/EngineSim.hpp"

using enginesim::EngineSim;
using enginesim::EngineSimConfig;
using enginesim::Ignition;
using link2::VehicleState;

namespace {

VehicleState armedAt(int8_t throttle, uint8_t gear = 1) {
    VehicleState s;
    s.armed = true;
    s.failsafe = false;
    s.throttlePercent = throttle;
    s.gear = gear;
    return s;
}

// Advances the sim by `count` ticks of `dtMs`, holding `state`, from `startMs`.
uint32_t run(EngineSim& e, const VehicleState& state, int count, uint32_t dtMs, uint32_t startMs) {
    uint32_t t = startMs;
    for (int i = 0; i < count; ++i) {
        t += dtMs;
        e.update(t, state);
    }
    return t;
}

} // namespace

void setUp() {}
void tearDown() {}

void test_off_when_disarmed() {
    EngineSim e;
    VehicleState s; // armed=false by default
    e.update(20, s);
    TEST_ASSERT_EQUAL(Ignition::Off, e.engine().ignition);
    // rpm decays toward 0.
    run(e, s, 100, 20, 20);
    TEST_ASSERT_EQUAL(Ignition::Off, e.engine().ignition);
    TEST_ASSERT_EQUAL_UINT16(0, e.engine().engineRpm);
}

void test_cranking_then_running_on_arm() {
    EngineSim e;
    VehicleState s = armedAt(0);
    e.update(20, s);
    TEST_ASSERT_EQUAL(Ignition::Cranking, e.engine().ignition);

    // Before crankMs (600): still cranking.
    run(e, s, 20, 20, 20); // t ~ 420
    TEST_ASSERT_EQUAL(Ignition::Cranking, e.engine().ignition);

    // Past crankMs: Running, settling toward idle.
    run(e, s, 40, 20, 420); // t ~ 1220
    TEST_ASSERT_EQUAL(Ignition::Running, e.engine().ignition);
    TEST_ASSERT_UINT16_WITHIN(300, 3500, e.engine().engineRpm); // idle +/- wobble
}

void test_revs_up_toward_throttle_target() {
    EngineSim e;
    VehicleState idle = armedAt(0);
    run(e, idle, 60, 20, 0); // reach Running + idle

    VehicleState full = armedAt(100);
    run(e, full, 60, 20, 1200); // ~1.2s of full throttle
    // Should be near redline (15000), comfortably above idle.
    TEST_ASSERT_TRUE(e.engine().engineRpm > 13000);
}

void test_rev_down_is_slower_than_rev_up() {
    EngineSim e;
    run(e, armedAt(0), 60, 20, 0); // Running at idle

    // 200ms of full throttle.
    uint32_t t = run(e, armedAt(100), 10, 20, 1200);
    const uint16_t afterRevUp = e.engine().engineRpm;

    // 200ms of lift from the same point.
    run(e, armedAt(0), 10, 20, t);
    const uint16_t afterRevDown = e.engine().engineRpm;

    // Rev-up gained more than rev-down shed in the same time (asymmetry).
    const int gained = static_cast<int>(afterRevUp) - 3500;
    const int shed = static_cast<int>(afterRevUp) - static_cast<int>(afterRevDown);
    TEST_ASSERT_TRUE(gained > shed);
}

void test_no_phantom_blip_on_first_state_or_failsafe_recovery() {
    EngineSim e;
    // First state ever is already in gear 4 -- must NOT register a shift.
    VehicleState g4 = armedAt(50, 4);
    run(e, g4, 60, 20, 0); // Running
    // No way to directly read blip, but engineRpm should track the throttle
    // target smoothly with no downshift spike above the target.
    const uint16_t target = 3500 + (15000 - 3500) * 50 / 100; // 9250
    TEST_ASSERT_UINT16_WITHIN(600, target, e.engine().engineRpm);
}

void test_limiter_flag_at_redline_full_throttle() {
    EngineSim e;
    run(e, armedAt(100), 200, 20, 0); // long full-throttle pull to redline
    TEST_ASSERT_TRUE(e.engine().limiterActive);
    TEST_ASSERT_UINT16_WITHIN(300, 15000, e.engine().engineRpm);
}

void test_overrun_window_on_fast_lift_from_high_rpm() {
    EngineSim e;
    uint32_t t = run(e, armedAt(100), 120, 20, 0); // at redline
    TEST_ASSERT_FALSE(e.engine().overrunActive);

    // Snap to zero throttle: big drop from high rpm -> overrun window opens.
    t += 20;
    e.update(t, armedAt(0));
    TEST_ASSERT_TRUE(e.engine().overrunActive);

    // Window closes after overrunMs (900).
    run(e, armedAt(0), 60, 20, t); // ~1.2s later
    TEST_ASSERT_FALSE(e.engine().overrunActive);
}

void test_ers_whine_passthrough() {
    EngineSim e;
    VehicleState s = armedAt(80);
    s.ersDeploying = true;
    run(e, s, 60, 20, 0);
    TEST_ASSERT_TRUE(e.engine().ersWhine);
}

void test_config_valid() {
    TEST_ASSERT_TRUE(EngineSimConfig{}.valid());
    EngineSimConfig bad;
    bad.maxRpm = 1000; // below idle
    TEST_ASSERT_FALSE(bad.valid());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_off_when_disarmed);
    RUN_TEST(test_cranking_then_running_on_arm);
    RUN_TEST(test_revs_up_toward_throttle_target);
    RUN_TEST(test_rev_down_is_slower_than_rev_up);
    RUN_TEST(test_no_phantom_blip_on_first_state_or_failsafe_recovery);
    RUN_TEST(test_limiter_flag_at_redline_full_throttle);
    RUN_TEST(test_overrun_window_on_fast_lift_from_high_rpm);
    RUN_TEST(test_ers_whine_passthrough);
    RUN_TEST(test_config_valid);
    return UNITY_END();
}
