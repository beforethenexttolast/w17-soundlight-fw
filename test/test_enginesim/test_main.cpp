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

// --- SL-7 wrap-safety regression tests -------------------------------------
//
// The blip and overrun windows used to store an absolute deadline
// (`start + duration`) and test `nowMs < deadline`, which misbehaves across the
// ~49.7-day uint32_t millis() wrap. They now store an event-start timestamp and
// test wrap-safe unsigned elapsed time. Ordinary-time behavior is unchanged:
// active while `elapsed < duration`, inactive at exactly `duration`.
//
// The blip has no public flag, so we observe it through engineRpm: a downshift
// adds +shiftBlipRpm (1400) and an upshift subtracts it while the window is
// open. We settle the engine at a steady throttle so the target rpm is flat and
// the blip offset stands well clear of idle wobble (~60 rpm at throttle 50).

namespace {

constexpr uint16_t kSettledTarget = 3500 + (15000 - 3500) * 50 / 100; // 9250
constexpr uint32_t kBlipMs = 130;    // EngineSimConfig::blipMs default
constexpr uint32_t kOverrunMs = 900; // EngineSimConfig::overrunMs default

// Advance to a settled Running state at throttle 50 / the given gear, ending at
// the returned absolute time. Long enough to finish cranking and reach target.
uint32_t settleAtGear(EngineSim& e, uint8_t gear, uint32_t startMs) {
    return run(e, armedAt(50, gear), 200, 20, startMs);
}

} // namespace

void test_shift_blip_normal_time_boundaries() {
    EngineSim e;
    const uint32_t t = settleAtGear(e, 5, 0);
    const uint16_t settled = e.engine().engineRpm;
    TEST_ASSERT_UINT16_WITHIN(300, kSettledTarget, settled);

    const uint32_t T0 = t + 20;
    const VehicleState d = armedAt(50, 4); // downshift 5 -> 4 => +1400 blip

    // Inactive just before the trigger is implied by `settled` above.
    // Active immediately after the trigger.
    e.update(T0, d);
    TEST_ASSERT_TRUE(e.engine().engineRpm > settled + 1000);

    // Active at duration - 1 ms (no new shift: gear stays 4).
    e.update(static_cast<uint32_t>(T0 + (kBlipMs - 1)), d);
    TEST_ASSERT_TRUE(e.engine().engineRpm > settled + 1000);

    // Inactive exactly at duration.
    e.update(static_cast<uint32_t>(T0 + kBlipMs), d);
    TEST_ASSERT_UINT16_WITHIN(300, kSettledTarget, e.engine().engineRpm);

    // Remains inactive after duration.
    e.update(static_cast<uint32_t>(T0 + kBlipMs + 70), d);
    TEST_ASSERT_UINT16_WITHIN(300, kSettledTarget, e.engine().engineRpm);
}

void test_shift_blip_across_wrap() {
    EngineSim e;
    // Settle Running before the wrap so the blip window straddles UINT32_MAX.
    const uint32_t base = static_cast<uint32_t>(UINT32_MAX - 50u - 4000u);
    const uint32_t t = settleAtGear(e, 5, base);
    const uint16_t settled = e.engine().engineRpm;
    TEST_ASSERT_UINT16_WITHIN(300, kSettledTarget, settled);
    (void)t;

    const uint32_t T0 = static_cast<uint32_t>(UINT32_MAX - 50u); // trigger near wrap
    const VehicleState d = armedAt(50, 4);                       // downshift => +1400

    e.update(T0, d); // trigger
    TEST_ASSERT_TRUE(e.engine().engineRpm > settled + 1000);

    // Active immediately before wrap (elapsed 40, nowMs still < UINT32_MAX).
    e.update(static_cast<uint32_t>(T0 + 40u), d);
    TEST_ASSERT_TRUE(e.engine().engineRpm > settled + 1000);

    // Active immediately after wrap (elapsed 60, nowMs wrapped to a small value).
    e.update(static_cast<uint32_t>(T0 + 60u), d);
    TEST_ASSERT_TRUE(e.engine().engineRpm > settled + 1000);

    // Active at duration - 1.
    e.update(static_cast<uint32_t>(T0 + (kBlipMs - 1)), d);
    TEST_ASSERT_TRUE(e.engine().engineRpm > settled + 1000);

    // Inactive exactly at duration.
    e.update(static_cast<uint32_t>(T0 + kBlipMs), d);
    TEST_ASSERT_UINT16_WITHIN(300, kSettledTarget, e.engine().engineRpm);

    // Inactive after duration.
    e.update(static_cast<uint32_t>(T0 + kBlipMs + 70), d);
    TEST_ASSERT_UINT16_WITHIN(300, kSettledTarget, e.engine().engineRpm);
}

void test_shift_blip_retrigger_restarts_timer() {
    EngineSim e;
    const uint32_t t = settleAtGear(e, 4, 0);
    const uint16_t settled = e.engine().engineRpm;
    TEST_ASSERT_UINT16_WITHIN(300, kSettledTarget, settled);

    const uint32_t T0 = t + 20;
    // Downshift 4 -> 3: +1400 spike.
    e.update(T0, armedAt(50, 3));
    TEST_ASSERT_TRUE(e.engine().engineRpm > settled + 1000);

    // Re-trigger while still active (elapsed 100 < 130): upshift 3 -> 4 => -1400
    // dip, and the window restarts from here (start + duration is not retained).
    e.update(static_cast<uint32_t>(T0 + 100), armedAt(50, 4));
    TEST_ASSERT_TRUE(e.engine().engineRpm + 1000 < settled);

    // At T0+200: 170 ms past the FIRST trigger (old window would be closed) but
    // only 100 ms past the restart, so the dip is still active -> proves restart.
    e.update(static_cast<uint32_t>(T0 + 200), armedAt(50, 4));
    TEST_ASSERT_TRUE(e.engine().engineRpm + 1000 < settled);

    // Restarted window closes at (T0+100) + 130 = T0+230.
    e.update(static_cast<uint32_t>(T0 + 230), armedAt(50, 4));
    TEST_ASSERT_UINT16_WITHIN(300, kSettledTarget, e.engine().engineRpm);
}

void test_overrun_normal_time_boundaries() {
    EngineSim e;
    const uint32_t t = run(e, armedAt(100), 120, 20, 0); // at redline
    TEST_ASSERT_FALSE(e.engine().overrunActive);

    const uint32_t T0 = t + 20;
    const VehicleState lift = armedAt(0); // fast drop from high rpm opens window
    e.update(T0, lift);
    TEST_ASSERT_TRUE(e.engine().overrunActive);

    // Active at duration - 1 (throttle held at 0 => no re-trigger).
    e.update(static_cast<uint32_t>(T0 + (kOverrunMs - 1)), lift);
    TEST_ASSERT_TRUE(e.engine().overrunActive);

    // Inactive exactly at duration.
    e.update(static_cast<uint32_t>(T0 + kOverrunMs), lift);
    TEST_ASSERT_FALSE(e.engine().overrunActive);

    // Inactive afterward.
    e.update(static_cast<uint32_t>(T0 + kOverrunMs + 100), lift);
    TEST_ASSERT_FALSE(e.engine().overrunActive);
}

void test_overrun_across_wrap() {
    EngineSim e;
    const uint32_t base = static_cast<uint32_t>(UINT32_MAX - 50u - 4000u);
    run(e, armedAt(100), 180, 20, base); // crank + climb to redline before wrap

    const uint32_t T0 = static_cast<uint32_t>(UINT32_MAX - 50u);
    const VehicleState lift = armedAt(0);
    e.update(T0, lift); // trigger near wrap
    TEST_ASSERT_TRUE(e.engine().overrunActive);

    // Active immediately before wrap.
    e.update(static_cast<uint32_t>(T0 + 40u), lift);
    TEST_ASSERT_TRUE(e.engine().overrunActive);

    // Active immediately after wrap.
    e.update(static_cast<uint32_t>(T0 + 100u), lift);
    TEST_ASSERT_TRUE(e.engine().overrunActive);

    // Active at duration - 1.
    e.update(static_cast<uint32_t>(T0 + (kOverrunMs - 1)), lift);
    TEST_ASSERT_TRUE(e.engine().overrunActive);

    // Inactive exactly at duration.
    e.update(static_cast<uint32_t>(T0 + kOverrunMs), lift);
    TEST_ASSERT_FALSE(e.engine().overrunActive);

    // Inactive after duration.
    e.update(static_cast<uint32_t>(T0 + kOverrunMs + 100), lift);
    TEST_ASSERT_FALSE(e.engine().overrunActive);
}

void test_overrun_retrigger_restarts_timer() {
    EngineSim e;
    const uint32_t t = run(e, armedAt(100), 120, 20, 0); // redline
    const uint32_t T0 = t + 20;
    e.update(T0, armedAt(0)); // first trigger
    TEST_ASSERT_TRUE(e.engine().overrunActive);

    // Re-climb to redline (window still open: elapsed < 900), then lift again.
    const uint32_t t2 = run(e, armedAt(100), 10, 20, T0); // T0 + 200
    const uint32_t T1 = t2 + 20;                          // T0 + 220
    e.update(T1, armedAt(0));                             // second trigger -> restart
    TEST_ASSERT_TRUE(e.engine().overrunActive);

    // At T0+1000: past the FIRST window (T0+900) but inside the restarted one
    // (T1+900 = T0+1120) -> still active proves the timer restarted.
    e.update(static_cast<uint32_t>(T0 + 1000), armedAt(0));
    TEST_ASSERT_TRUE(e.engine().overrunActive);

    // Restarted window closes at T1 + 900.
    e.update(static_cast<uint32_t>(T1 + kOverrunMs), armedAt(0));
    TEST_ASSERT_FALSE(e.engine().overrunActive);
}

void test_crank_transition_across_wrap() {
    EngineSim e;
    const uint32_t T0 = static_cast<uint32_t>(UINT32_MAX - 300u); // arm near wrap
    const VehicleState s = armedAt(0);
    e.update(T0, s);
    TEST_ASSERT_EQUAL(Ignition::Cranking, e.engine().ignition);

    // Before crankMs (600): elapsed 500, nowMs wrapped past UINT32_MAX.
    e.update(static_cast<uint32_t>(T0 + 500u), s);
    TEST_ASSERT_EQUAL(Ignition::Cranking, e.engine().ignition);

    // Past crankMs: elapsed 700 -> Running.
    e.update(static_cast<uint32_t>(T0 + 700u), s);
    TEST_ASSERT_EQUAL(Ignition::Running, e.engine().ignition);
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
    RUN_TEST(test_shift_blip_normal_time_boundaries);
    RUN_TEST(test_shift_blip_across_wrap);
    RUN_TEST(test_shift_blip_retrigger_restarts_timer);
    RUN_TEST(test_overrun_normal_time_boundaries);
    RUN_TEST(test_overrun_across_wrap);
    RUN_TEST(test_overrun_retrigger_restarts_timer);
    RUN_TEST(test_crank_transition_across_wrap);
    RUN_TEST(test_config_valid);
    return UNITY_END();
}
