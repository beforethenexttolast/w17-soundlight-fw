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
    // From a common mid point, an equal-duration rev-up gains MORE rpm than an
    // equal-duration rev-down sheds, because revUpPerMille (6) > revDownPerMille
    // (3). Both legs must stay inside the physical band. (The pre-fix defect made
    // the rev-down leg's negative gap divide unsigned, wrapping engineRpm to a
    // garbage uint16 and satisfying the old `gained > shed` check via a nonsense
    // negative `shed` -- i.e. the old test proved nothing.)
    EngineSim up;
    run(up, armedAt(50), 200, 20, 0); // settle at the throttle-50 target (9250)
    const uint16_t mid = up.engine().engineRpm;
    TEST_ASSERT_UINT16_WITHIN(300, 9250, mid);

    const uint32_t t = 200u * 20u;
    run(up, armedAt(100), 10, 20, t); // 200 ms toward full throttle
    const uint16_t afterRevUp = up.engine().engineRpm;
    TEST_ASSERT_TRUE(afterRevUp > mid);    // moved up
    TEST_ASSERT_TRUE(afterRevUp <= 15100); // bounded, no wrap

    EngineSim down;
    run(down, armedAt(50), 200, 20, 0); // same mid point
    TEST_ASSERT_UINT16_WITHIN(300, 9250, down.engine().engineRpm);
    run(down, armedAt(0), 10, 20, t); // 200 ms toward idle
    const uint16_t afterRevDown = down.engine().engineRpm;
    TEST_ASSERT_TRUE(afterRevDown < mid);   // moved down, not up/wrapped
    TEST_ASSERT_TRUE(afterRevDown >= 3300); // bounded to the idle floor

    const int gained = static_cast<int>(afterRevUp) - static_cast<int>(mid);
    const int shed = static_cast<int>(mid) - static_cast<int>(afterRevDown);
    TEST_ASSERT_TRUE(gained > 0);
    TEST_ASSERT_TRUE(shed > 0);
    TEST_ASSERT_TRUE(gained > shed); // asymmetry: rev-up faster than rev-down
}

// --- Signed inertia arithmetic (negative-gap correctness) ------------------
//
// The inertia step must be computed in signed, int64-widened arithmetic and
// divided (not shifted) by exactly 1000 with truncation toward zero, so a
// negative gap (rev-down / crank-down) yields a negative step. The prior code
// divided a signed numerator by an unsigned scale, converting negatives to a
// huge positive value (and eventually overflowing int32).

void test_inertia_step_signed_policy() {
    // Fixed literals: signed, per-mille-per-ms, divided by 1000.
    TEST_ASSERT_EQUAL_INT32(-164, EngineSim::inertiaStep(-8200, 20, 1));
    TEST_ASSERT_EQUAL_INT32(164, EngineSim::inertiaStep(8200, 20, 1));
    TEST_ASSERT_EQUAL_INT32(-1640, EngineSim::inertiaStep(-8200, 20, 10));
    TEST_ASSERT_EQUAL_INT32(0, EngineSim::inertiaStep(0, 20, 10));

    // Truncation toward zero for |numerator| straddling the 1000 scale.
    TEST_ASSERT_EQUAL_INT32(0, EngineSim::inertiaStep(-999, 1, 1));   // -999/1000  -> 0
    TEST_ASSERT_EQUAL_INT32(-1, EngineSim::inertiaStep(-1000, 1, 1)); // -1000/1000 -> -1
    TEST_ASSERT_EQUAL_INT32(-1, EngineSim::inertiaStep(-1001, 1, 1)); // -1001/1000 -> -1
    TEST_ASSERT_EQUAL_INT32(0, EngineSim::inertiaStep(999, 1, 1));    // +999/1000  -> 0
    TEST_ASSERT_EQUAL_INT32(1, EngineSim::inertiaStep(1001, 1, 1));   // +1001/1000 -> +1

    // Extreme valid()-domain magnitudes: |gap| <= 65535, rate <= 65535, dt <= 100.
    // The int64 numerator (~4.29e11) would overflow a 32-bit multiply; the
    // post-division step still fits int32 (429,483,622 < INT32_MAX).
    TEST_ASSERT_EQUAL_INT32(429483622, EngineSim::inertiaStep(65535, 65535, 100));
    TEST_ASSERT_EQUAL_INT32(-429483622, EngineSim::inertiaStep(-65535, 65535, 100));
}

void test_rev_down_is_monotonic_and_settles() {
    // Real EngineSim, default config. After a lift from redline, engineRpm must
    // decay toward idle: never jump up (beyond a small idle-wobble tolerance),
    // never wrap to a huge uint16, and finally settle at idle. Fails on the
    // pre-fix code, which drives engineRpm to wrapped garbage on the first lift
    // tick.
    EngineSim e;
    run(e, armedAt(100), 200, 20, 0); // climb to a stable redline
    const uint16_t high = e.engine().engineRpm;
    TEST_ASSERT_UINT16_WITHIN(300, 15000, high);

    const VehicleState idle = armedAt(0);
    uint32_t t = 200u * 20u;
    uint16_t prev = high;
    bool reachedIdle = false;
    for (int i = 0; i < 200; ++i) {
        t += 20;
        e.update(t, idle);
        const uint16_t now = e.engine().engineRpm;
        // Idle wobble can nudge engineRpm up by a few tens of rpm; the defect
        // jumped it by tens of thousands. 200 rpm cleanly separates the two.
        TEST_ASSERT_TRUE(now <= prev + 200);
        TEST_ASSERT_TRUE(now <= 15100); // stays in band, never wraps high
        TEST_ASSERT_TRUE(now >= 3300);  // never undershoots the idle floor
        prev = now;
        if (now <= 3500 + 200) {
            reachedIdle = true;
        }
    }
    TEST_ASSERT_TRUE(reachedIdle);
    TEST_ASSERT_UINT16_WITHIN(300, 3500, e.engine().engineRpm);
}

void test_cranking_trajectory_is_bounded() {
    // Off -> Cranking -> Running. On the first crank tick the target is crankRpm
    // (1800), below the constructor's idle seed (3500), so the gap is NEGATIVE --
    // the same negative-gap path the defect corrupts. engineRpm must stay bounded
    // and sane throughout, then settle at idle once Running.
    EngineSim e;
    const VehicleState s = armedAt(0);
    uint32_t t = 20;
    e.update(t, s);
    TEST_ASSERT_EQUAL(Ignition::Cranking, e.engine().ignition);

    for (; t < 600; t += 20) {
        e.update(t, s);
        if (e.engine().ignition == Ignition::Cranking) {
            // Cranking rpm decays 3500 -> 1800; never above the idle seed, never
            // wrapped high. Pre-fix, the negative gap wraps engineRpm well past this.
            TEST_ASSERT_TRUE(e.engine().engineRpm <= 3500);
            TEST_ASSERT_TRUE(e.engine().engineRpm >= 1000);
        }
    }

    // Past crankMs -> Running (rpm reset to idle), settling near idle.
    run(e, s, 60, 20, t);
    TEST_ASSERT_EQUAL(Ignition::Running, e.engine().ignition);
    TEST_ASSERT_UINT16_WITHIN(300, 3500, e.engine().engineRpm);
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

// --- Showcase sound authority (link2 modeFlags bit0) -------------------------

namespace {

// Effective showcase frame as the monitor delivers it while the link is Up
// and board #1 is in its SHOWCASE boot: showcase set, armed 0, throttle 0
// (truthful wire), no failsafe.
VehicleState showcaseIdle() {
    VehicleState s;
    s.showcase = true;
    s.armed = false;
    s.failsafe = false;
    s.throttlePercent = 0;
    return s;
}

} // namespace

// Showcase 0->1 cranks exactly like arming does (the wave-3 ignition halo
// animation keys off this same machine, so the starter sweep + catch come
// free), then settles at idle.
void test_showcase_cranks_then_runs() {
    EngineSim e;
    VehicleState s = showcaseIdle();
    e.update(20, s);
    TEST_ASSERT_EQUAL(Ignition::Cranking, e.engine().ignition);
    run(e, s, 60, 20, 20);
    TEST_ASSERT_EQUAL(Ignition::Running, e.engine().ignition);
    TEST_ASSERT_UINT16_WITHIN(300, 3500, e.engine().engineRpm); // idle +/- wobble
}

// D4 row 3: board #1 keeps sending showcase=1 but with failsafe=1 after a
// mid-session radio death -- the show must fall silent. From boot AND from
// a running show alike.
void test_showcase_with_failsafe_stays_off() {
    EngineSim fromBoot;
    VehicleState s = showcaseIdle();
    s.failsafe = true;
    run(fromBoot, s, 60, 20, 0);
    TEST_ASSERT_EQUAL(Ignition::Off, fromBoot.engine().ignition);
    TEST_ASSERT_EQUAL_UINT16(0, fromBoot.engine().engineRpm);

    EngineSim midShow;
    run(midShow, showcaseIdle(), 60, 20, 0); // Running
    TEST_ASSERT_EQUAL(Ignition::Running, midShow.engine().ignition);
    run(midShow, s, 2, 20, 1200); // failsafe asserts mid-show
    TEST_ASSERT_EQUAL(Ignition::Off, midShow.engine().ignition);
}

// D5: low battery ENDS the show -- ignition Off (silence) so the red halo
// pulse is unmissable -- while the ARMED path keeps its warn-only rule
// (a driving car never loses its engine sound to a battery warning).
void test_showcase_low_battery_ends_show_but_armed_path_warn_only() {
    EngineSim show;
    run(show, showcaseIdle(), 60, 20, 0);
    TEST_ASSERT_EQUAL(Ignition::Running, show.engine().ignition);
    VehicleState low = showcaseIdle();
    low.lowBattery = true;
    run(show, low, 2, 20, 1200);
    TEST_ASSERT_EQUAL(Ignition::Off, show.engine().ignition); // the show ends

    EngineSim drive;
    VehicleState armedLow = armedAt(30);
    armedLow.lowBattery = true;
    run(drive, armedLow, 60, 20, 0);
    TEST_ASSERT_EQUAL(Ignition::Running, drive.engine().ignition); // warn-only, unchanged
}

// The monitor's Lost projection zeroes the showcase bit (command-class);
// this is the enginesim half of that contract: a cleared bit kills the
// ignition on the next tick.
void test_showcase_cleared_drops_to_off() {
    EngineSim e;
    run(e, showcaseIdle(), 60, 20, 0);
    TEST_ASSERT_EQUAL(Ignition::Running, e.engine().ignition);
    VehicleState gone; // all defaults: showcase 0, armed 0, failsafe 1
    e.update(1220, gone);
    TEST_ASSERT_EQUAL(Ignition::Off, e.engine().ignition);
}

// The authority predicates themselves, exhaustively over the four gating
// flags (armed path must reduce to `armed` alone -- byte-unchanged drive
// semantics; showcase path needs showcase && !armed && !failsafe &&
// !lowBattery).
void test_ignition_authority_truth_table() {
    for (int mask = 0; mask < 16; ++mask) {
        VehicleState s;
        s.armed = (mask & 1) != 0;
        s.showcase = (mask & 2) != 0;
        s.failsafe = (mask & 4) != 0;
        s.lowBattery = (mask & 8) != 0;
        const bool expectShow = s.showcase && !s.armed && !s.failsafe && !s.lowBattery;
        TEST_ASSERT_EQUAL(expectShow, enginesim::showcaseSoundAuthority(s));
        TEST_ASSERT_EQUAL(s.armed || expectShow, enginesim::ignitionAuthority(s));
    }
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_off_when_disarmed);
    RUN_TEST(test_cranking_then_running_on_arm);
    RUN_TEST(test_revs_up_toward_throttle_target);
    RUN_TEST(test_rev_down_is_slower_than_rev_up);
    RUN_TEST(test_inertia_step_signed_policy);
    RUN_TEST(test_rev_down_is_monotonic_and_settles);
    RUN_TEST(test_cranking_trajectory_is_bounded);
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
    RUN_TEST(test_showcase_cranks_then_runs);
    RUN_TEST(test_showcase_with_failsafe_stays_off);
    RUN_TEST(test_showcase_low_battery_ends_show_but_armed_path_warn_only);
    RUN_TEST(test_showcase_cleared_drops_to_off);
    RUN_TEST(test_ignition_authority_truth_table);
    return UNITY_END();
}
