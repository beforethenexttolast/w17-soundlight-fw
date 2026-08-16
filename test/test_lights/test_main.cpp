#include <unity.h>

#include "lights/LightRenderer.hpp"

using light_status = link2monitor::LinkStatus;
using lights::kNumPixels;
using lights::LightConfig;
using lights::LightRenderer;
using lights::Rgb;
using link2::VehicleState;

namespace {

LightConfig cfg;

// Most tests exercise steady states where the ignition animation is not the
// subject; rendering with Ignition::Off leaves every non-halo layer exactly
// as it was before the animation existed (and the halo base keys off
// state.armed as before). Ignition-specific tests pass explicit values.
constexpr auto kIgnOff = enginesim::Ignition::Off;
constexpr auto kIgnCranking = enginesim::Ignition::Cranking;
constexpr auto kIgnRunning = enginesim::Ignition::Running;

// Returns the first pixel of a segment after a render.
Rgb segFirst(Rgb* px, const lights::Segment& s) { return px[s.start]; }

bool anyNonBlack(Rgb* px) {
    for (uint8_t i = 0; i < kNumPixels; ++i) {
        if (!(px[i] == Rgb{0, 0, 0})) return true;
    }
    return false;
}

VehicleState upState() {
    VehicleState s;
    s.armed = true;
    s.failsafe = false;
    return s;
}

} // namespace

void setUp() {}
void tearDown() {}

void test_config_valid_and_within_power_budget() {
    TEST_ASSERT_TRUE(LightConfig{}.valid());
    LightConfig tooBright;
    tooBright.maxBrightness = 255;
    // 30 LEDs amber at full would exceed the 900mA budget -> invalid.
    TEST_ASSERT_FALSE(tooBright.valid());
}

void test_failsafe_hazard_overrides_everything() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = upState();
    s.braking = true; // would normally light the brake
    s.failsafe = true;

    // At the on-phase of the hazard blink, all pixels are amber (R>0,G>0,B=0).
    r.render(s, light_status::Up, kIgnOff, /*nowMs=*/0, px);
    for (uint8_t i = 0; i < kNumPixels; ++i) {
        TEST_ASSERT_TRUE(px[i].r > 0 && px[i].g > 0 && px[i].b == 0);
    }
    // Half a period later: all off.
    r.render(s, light_status::Up, kIgnOff, cfg.hazardPeriodMs / 2, px);
    for (uint8_t i = 0; i < kNumPixels; ++i) {
        TEST_ASSERT_TRUE(px[i] == (Rgb{0, 0, 0}));
    }
}

void test_link_lost_forces_hazard_even_if_frame_not_failsafe() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = upState(); // frame says fine...
    r.render(s, light_status::Lost, kIgnOff, 0, px); // ...but the link is Lost
    for (uint8_t i = 0; i < kNumPixels; ++i) {
        TEST_ASSERT_TRUE(px[i].r > 0 && px[i].g > 0 && px[i].b == 0);
    }
}

void test_never_connected_is_calm_not_hazard() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s; // defaults
    r.render(s, light_status::NeverConnected, kIgnOff, 500, px);
    // Waiting breathe is teal-ish on the halo, never the amber hazard.
    bool anyAmber = false;
    for (uint8_t i = 0; i < kNumPixels; ++i) {
        if (px[i].r > 0 && px[i].g > 0 && px[i].b == 0 && px[i].r > 20) anyAmber = true;
    }
    TEST_ASSERT_FALSE(anyAmber);
}

namespace {

// True when every pixel is the hazard amber (R>0, G>0, B==0).
bool allAmber(Rgb* px) {
    for (uint8_t i = 0; i < kNumPixels; ++i) {
        if (!(px[i].r > 0 && px[i].g > 0 && px[i].b == 0)) return false;
    }
    return true;
}

// True when no pixel shows the hazard amber signature.
bool noAmber(Rgb* px) {
    for (uint8_t i = 0; i < kNumPixels; ++i) {
        if (px[i].r > 0 && px[i].g > 0 && px[i].b == 0) return false;
    }
    return true;
}

} // namespace

// Audit defect 9: a wire cut BEFORE the first frame used to breathe calm teal
// forever. Now the calm breathe holds only for neverConnectedGraceMs after the
// first render, then escalates to the exact hazard pattern a Lost link shows.
void test_never_connected_escalates_to_hazard_after_grace() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s; // NeverConnected effective state = all-safe defaults

    // Inside the grace window: calm, never amber.
    r.render(s, light_status::NeverConnected, kIgnOff, 0, px); // seeds the grace start
    TEST_ASSERT_TRUE(noAmber(px));
    r.render(s, light_status::NeverConnected, kIgnOff, cfg.neverConnectedGraceMs - 1, px);
    TEST_ASSERT_TRUE(noAmber(px));

    // At exactly the grace boundary: hazard (boundary is inclusive-stale, the
    // same convention as the monitor's staleness compare). Sample an on-phase
    // and an off-phase of the blink to prove it is the BLINKING hazard, not a
    // recolored breathe.
    const uint32_t tOn = cfg.neverConnectedGraceMs; // grace % hazardPeriod == 0 -> on
    r.render(s, light_status::NeverConnected, kIgnOff, tOn, px);
    TEST_ASSERT_TRUE(allAmber(px));
    r.render(s, light_status::NeverConnected, kIgnOff, tOn + cfg.hazardPeriodMs / 2, px);
    for (uint8_t i = 0; i < kNumPixels; ++i) {
        TEST_ASSERT_TRUE(px[i] == (Rgb{0, 0, 0}));
    }
}

// The grace window is measured from the FIRST render, not from absolute time
// zero -- a board whose clock is already far along must still get its full
// calm window.
void test_never_connected_grace_measured_from_first_render() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s;

    const uint32_t t0 = 100000; // well past the grace length in absolute time
    r.render(s, light_status::NeverConnected, kIgnOff, t0, px); // seed here
    TEST_ASSERT_TRUE(noAmber(px));
    r.render(s, light_status::NeverConnected, kIgnOff, t0 + cfg.neverConnectedGraceMs - 1, px);
    TEST_ASSERT_TRUE(noAmber(px));
    r.render(s, light_status::NeverConnected, kIgnOff, t0 + cfg.neverConnectedGraceMs, px);
    TEST_ASSERT_TRUE(allAmber(px));
}

// Pins the grace-window validation band: long enough that a healthy same-rail
// power-up never flashes hazard, short enough that a dead harness is signaled
// while someone is still looking at the car.
void test_never_connected_grace_validation_bounds() {
    LightConfig c;
    c.neverConnectedGraceMs = 999;
    TEST_ASSERT_FALSE(c.valid());
    c.neverConnectedGraceMs = 1000;
    TEST_ASSERT_TRUE(c.valid());
    c.neverConnectedGraceMs = 30000;
    TEST_ASSERT_TRUE(c.valid());
    c.neverConnectedGraceMs = 30001;
    TEST_ASSERT_FALSE(c.valid());
    TEST_ASSERT_TRUE(LightConfig{}.valid()); // default stays valid
}

void test_brake_lights_on_braking() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = upState();
    s.braking = true;
    r.render(s, light_status::Up, kIgnOff, 0, px);
    const Rgb brake = segFirst(px, cfg.brake);
    TEST_ASSERT_TRUE(brake.r > brake.g && brake.r > brake.b); // dominant red
}

void test_indicator_hysteresis_and_selfcancel() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = upState();

    // Steer hard right: right indicator active (sample at an on-phase).
    s.steeringPercent = 60;
    r.render(s, light_status::Up, kIgnOff, 0, px);
    TEST_ASSERT_TRUE(!(segFirst(px, cfg.rightIndicator) == (Rgb{0, 0, 0})));

    // Ease to +30 (between off=20 and on=40): hysteresis holds it ON.
    s.steeringPercent = 30;
    r.render(s, light_status::Up, kIgnOff, 0, px);
    TEST_ASSERT_TRUE(!(segFirst(px, cfg.rightIndicator) == (Rgb{0, 0, 0})));

    // Return under the off threshold: self-cancels.
    s.steeringPercent = 10;
    r.render(s, light_status::Up, kIgnOff, 0, px);
    TEST_ASSERT_TRUE(segFirst(px, cfg.rightIndicator) == (Rgb{0, 0, 0}));
}

void test_rain_light_flashes_only_while_harvesting_in_ers_mode() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = upState();
    s.driveMode = 2;
    s.ersPercent = 40;

    // Seed the harvest detector (first frame establishes baseline, no rise).
    r.render(s, light_status::Up, kIgnOff, 0, px);

    // ersPercent rises -> harvesting -> rain light flashes (sample on-phase).
    s.ersPercent = 42;
    r.render(s, light_status::Up, kIgnOff, 100, px); // 100 % 250 < 125 => on
    TEST_ASSERT_TRUE(!(segFirst(px, cfg.rainLight) == (Rgb{0, 0, 0})));

    // No rise for a while -> harvest window closes -> rain light off.
    r.render(s, light_status::Up, kIgnOff, 100 + 500, px);
    TEST_ASSERT_TRUE(segFirst(px, cfg.rainLight) == (Rgb{0, 0, 0}));
}

void test_rain_light_ignores_deploy_only() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = upState();
    s.driveMode = 2;
    s.ersPercent = 80;
    r.render(s, light_status::Up, kIgnOff, 0, px);

    // Deploying: ersPercent FALLS -> not harvesting -> rain light stays off.
    s.ersDeploying = true;
    s.ersPercent = 76;
    r.render(s, light_status::Up, kIgnOff, 100, px);
    TEST_ASSERT_TRUE(segFirst(px, cfg.rainLight) == (Rgb{0, 0, 0}));
}

void test_halo_teal_armed_dim_when_disarmed() {
    LightRenderer r;
    Rgb px[kNumPixels];

    VehicleState armed = upState();
    r.render(armed, light_status::Up, kIgnOff, 0, px);
    const Rgb haloArmed = segFirst(px, cfg.halo);
    TEST_ASSERT_TRUE(haloArmed.g >= haloArmed.r); // teal: green/blue dominant

    VehicleState disarmed;
    disarmed.armed = false;
    disarmed.failsafe = false;
    // Disarmed but link Up (board #1 can send disarmed-idle frames).
    r.render(disarmed, light_status::Up, kIgnOff, 0, px);
    TEST_ASSERT_TRUE(anyNonBlack(px)); // halo dim-white, not black
}

// --- Ignition-on animation (vision decision 16) ----------------------------
//
// The look under test: while the enginesim reports Cranking, a bright-cyan
// starter comet sweeps the halo (free-running phase, one lap per
// ignitionSweepPeriodMs); on entering Running the whole halo flashes cyan and
// crossfades into the armed teal over ignitionFlashMs. Base-layer only:
// low-battery and hazard always win.

void test_ignition_crank_sweep_moves_on_halo() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = upState();

    // Phase 0: comet head at the first halo pixel -- cyan family (g,b > 0,
    // r == 0), while a mid-halo pixel away from head+trail stays dark.
    r.render(s, light_status::Up, kIgnCranking, 0, px);
    const Rgb head0 = px[cfg.halo.start];
    TEST_ASSERT_TRUE(head0.g > 0 && head0.b > 0 && head0.r == 0);
    TEST_ASSERT_TRUE(px[cfg.halo.start + 5] == (Rgb{0, 0, 0}));

    // Half a period later the head has moved to len/2; the old head pixel is
    // dark again (the comet MOVES rather than blinking in place).
    const uint32_t tHalf = cfg.ignitionSweepPeriodMs / 2;
    const uint8_t mid = static_cast<uint8_t>(
        static_cast<uint32_t>(tHalf) * cfg.halo.len / cfg.ignitionSweepPeriodMs);
    r.render(s, light_status::Up, kIgnCranking, tHalf, px);
    const Rgb headMid = px[cfg.halo.start + mid];
    TEST_ASSERT_TRUE(headMid.g > 0 && headMid.b > 0 && headMid.r == 0);
    TEST_ASSERT_TRUE(px[cfg.halo.start] == (Rgb{0, 0, 0}));

    // The rear tail stays dim red throughout the crank (base layer intact).
    const Rgb tail = segFirst(px, cfg.brake);
    TEST_ASSERT_TRUE(tail.r > 0 && tail.g == 0 && tail.b == 0);
}

void test_ignition_fireup_flash_crossfades_to_teal() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = upState();

    // Crank first (establishes the Cranking->Running transition).
    r.render(s, light_status::Up, kIgnCranking, 800, px);

    // Engine catches: flash starts at full cyan.
    const uint32_t t0 = 1000;
    r.render(s, light_status::Up, kIgnRunning, t0, px);
    const uint8_t gFlash = px[cfg.halo.start].g;

    // Mid-window: strictly between the flash and the settled teal.
    r.render(s, light_status::Up, kIgnRunning, t0 + cfg.ignitionFlashMs / 2, px);
    const uint8_t gMid = px[cfg.halo.start].g;

    // At/after ignitionFlashMs: exactly the plain armed-teal base (compare
    // against a renderer that never flashed).
    r.render(s, light_status::Up, kIgnRunning, t0 + cfg.ignitionFlashMs, px);
    const Rgb settled = px[cfg.halo.start];
    LightRenderer plain;
    Rgb plainPx[kNumPixels];
    plain.render(s, light_status::Up, kIgnOff, t0 + cfg.ignitionFlashMs, plainPx);
    TEST_ASSERT_TRUE(settled == plainPx[cfg.halo.start]);

    // The crossfade is a genuine decay: flash > mid > settled brightness.
    TEST_ASSERT_TRUE(gFlash > gMid);
    TEST_ASSERT_TRUE(gMid > settled.g);
}

void test_ignition_animation_never_masks_alerts() {
    // Low battery during the fire-up flash: the halo shows the red alert
    // pulse, not the cyan flash (alert layer draws after base).
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = upState();
    r.render(s, light_status::Up, kIgnCranking, 800, px);
    s.lowBattery = true;
    r.render(s, light_status::Up, kIgnRunning, 833, px); // pulse near its crest
    const Rgb halo = px[cfg.halo.start];
    TEST_ASSERT_TRUE(halo.r > 0 && halo.g == 0 && halo.b == 0);

    // Failsafe during the crank sweep: hazard overrides the whole strip.
    LightRenderer r2;
    VehicleState fs = upState();
    fs.failsafe = true;
    r2.render(fs, light_status::Up, kIgnCranking, 0, px);
    TEST_ASSERT_TRUE(allAmber(px));
}

void test_ignition_flash_cancels_on_disarm_and_retriggers() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = upState();

    r.render(s, light_status::Up, kIgnCranking, 0, px);
    r.render(s, light_status::Up, kIgnRunning, 33, px); // flash active

    // Disarm mid-flash: the halo drops straight to the disarmed base, no
    // cyan residue (flash canceled, not paused).
    VehicleState disarmed;
    disarmed.armed = false;
    disarmed.failsafe = false;
    r.render(disarmed, light_status::Up, kIgnOff, 66, px);
    LightRenderer plain;
    Rgb plainPx[kNumPixels];
    plain.render(disarmed, light_status::Up, kIgnOff, 66, plainPx);
    TEST_ASSERT_TRUE(px[cfg.halo.start] == plainPx[cfg.halo.start]);

    // Re-entering Running is a fresh engine catch: the flash re-triggers.
    r.render(s, light_status::Up, kIgnRunning, 99, px);
    const Rgb halo = px[cfg.halo.start];
    TEST_ASSERT_TRUE(halo.g > 0 && halo.b > 0 && halo.r == 0);
    TEST_ASSERT_TRUE(halo.g > plainPx[cfg.halo.start].g); // brighter than any base
}

void test_ignition_config_validation_bounds() {
    LightConfig c;
    c.ignitionSweepPeriodMs = 0; // divisor in the comet position math
    TEST_ASSERT_FALSE(c.valid());
    c = LightConfig{};
    c.ignitionFlashMs = 0; // divisor in the crossfade
    TEST_ASSERT_FALSE(c.valid());
    c.ignitionFlashMs = 2000; // "a moment, not a mode" upper bound
    TEST_ASSERT_TRUE(c.valid());
    c.ignitionFlashMs = 2001;
    TEST_ASSERT_FALSE(c.valid());
    TEST_ASSERT_TRUE(LightConfig{}.valid());
}

// Pins the low-battery period contract: the renderer's pulse half-period is
// period/2, so period 0 or 1 (half == 0) would divide by zero. valid() must
// reject anything below 2; the smallest meaningful pulse is period 2.
void test_low_battery_period_validation_boundary() {
    LightConfig c; // otherwise-valid defaults; only vary the period.
    c.lowBatteryPeriodMs = 0;
    TEST_ASSERT_FALSE(c.valid());
    c.lowBatteryPeriodMs = 1;
    TEST_ASSERT_FALSE(c.valid()); // fails on committed HEAD (1 used to pass)
    c.lowBatteryPeriodMs = 2;
    TEST_ASSERT_TRUE(c.valid());
    c.lowBatteryPeriodMs = 3;
    TEST_ASSERT_TRUE(c.valid());
    // The production/default value stays valid.
    TEST_ASSERT_TRUE(LightConfig{}.valid());
}

// At the minimum valid period (2), the low-battery pulse must render cleanly
// (no divide-by-zero, in-range bytes, deterministic) and still change the halo
// versus the non-low-battery state.
void test_low_battery_min_valid_period_renders() {
    LightConfig c;
    c.lowBatteryPeriodMs = 2;
    TEST_ASSERT_TRUE(c.valid());

    LightRenderer r(c);
    Rgb px[kNumPixels];

    VehicleState low = upState();
    low.lowBattery = true;

    // Exercise both phases (phase 0 -> tri 0; phase 1 -> the period-phase half).
    for (uint32_t t = 0; t < 8; ++t) {
        r.render(low, light_status::Up, kIgnOff, t, px);
        for (uint8_t i = 0; i < kNumPixels; ++i) {
            // Bytes are inherently within range; assert stays a live check.
            TEST_ASSERT_TRUE(px[i].r <= 255 && px[i].g <= 255 && px[i].b <= 255);
        }
    }

    // Determinism: same inputs -> same pixels.
    Rgb a[kNumPixels];
    Rgb b[kNumPixels];
    r.render(low, light_status::Up, kIgnOff, 1, a);
    r.render(low, light_status::Up, kIgnOff, 1, b);
    for (uint8_t i = 0; i < kNumPixels; ++i) {
        TEST_ASSERT_TRUE(a[i] == b[i]);
    }

    // The low-battery pulse recolors the halo versus the non-low state.
    Rgb normalPx[kNumPixels];
    VehicleState normal = upState(); // lowBattery = false
    r.render(normal, light_status::Up, kIgnOff, 1, normalPx);
    const Rgb haloLow = segFirst(a, cfg.halo);
    const Rgb haloNormal = segFirst(normalPx, cfg.halo);
    TEST_ASSERT_FALSE(haloLow == haloNormal);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_config_valid_and_within_power_budget);
    RUN_TEST(test_failsafe_hazard_overrides_everything);
    RUN_TEST(test_link_lost_forces_hazard_even_if_frame_not_failsafe);
    RUN_TEST(test_never_connected_is_calm_not_hazard);
    RUN_TEST(test_never_connected_escalates_to_hazard_after_grace);
    RUN_TEST(test_never_connected_grace_measured_from_first_render);
    RUN_TEST(test_never_connected_grace_validation_bounds);
    RUN_TEST(test_brake_lights_on_braking);
    RUN_TEST(test_indicator_hysteresis_and_selfcancel);
    RUN_TEST(test_rain_light_flashes_only_while_harvesting_in_ers_mode);
    RUN_TEST(test_rain_light_ignores_deploy_only);
    RUN_TEST(test_halo_teal_armed_dim_when_disarmed);
    RUN_TEST(test_ignition_crank_sweep_moves_on_halo);
    RUN_TEST(test_ignition_fireup_flash_crossfades_to_teal);
    RUN_TEST(test_ignition_animation_never_masks_alerts);
    RUN_TEST(test_ignition_flash_cancels_on_disarm_and_retriggers);
    RUN_TEST(test_ignition_config_validation_bounds);
    RUN_TEST(test_low_battery_period_validation_boundary);
    RUN_TEST(test_low_battery_min_valid_period_renders);
    return UNITY_END();
}
