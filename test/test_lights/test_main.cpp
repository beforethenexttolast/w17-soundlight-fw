#include <unity.h>

#include <cmath>

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

// The gamma curve moved from a runtime table built with __builtin_pow into a
// constexpr LUT in the header (one copy, shared with the power budget). Pin
// every entry against the libm reference the old table used, so "the numbers
// did not move" is a checked fact rather than a claim in a commit message.
void test_gamma_lut_matches_the_libm_reference() {
    for (int i = 0; i < 256; ++i) {
        const double g = 255.0 * std::pow(i / 255.0, 2.2);
        const int expected = static_cast<int>(g + 0.5);
        TEST_ASSERT_EQUAL_INT(expected, static_cast<int>(lights::kGamma.v[i]));
    }
}

// sl:safety-1 (1): the cap is applied BEFORE gamma, which is why 110 reads as
// "~43%" but drives a full channel at only 40/255. Pin the arithmetic the
// header comments and docs/SIMULATION.md now state, so a future edit cannot
// quietly make the prose false again.
void test_cap_is_applied_before_gamma_not_after() {
    const uint8_t cap = LightConfig{}.maxBrightness;
    TEST_ASSERT_EQUAL_UINT8(110, cap);

    // Cap-then-gamma (what the renderer does): gamma(255 * 110/255) = gamma(110).
    TEST_ASSERT_EQUAL_UINT8(lights::kGamma.v[110], lights::renderedDuty(255, cap));
    TEST_ASSERT_EQUAL_UINT8(40, lights::renderedDuty(255, cap));

    // Gamma-then-cap (what the header used to claim) would be far brighter --
    // 255 * 110/255 = 110 -- so the two orders are not a wording detail.
    TEST_ASSERT_TRUE(lights::renderedDuty(255, cap) < 110);

    // The pre-gamma budget model this replaced over-counted its own
    // two-primary worst case by 2.8x: 2*20*110/255 = 17 mA/LED (510 mA over
    // the strip) against the gamma-aware 2*20*40/255 = 6 mA/LED (180 mA) --
    // and it over-counted the actual all-amber hazard draw (kAmber =
    // {255,90,0}, both channels through cap+gamma: 510 mA against
    // ~104 mA post-gamma) by ~5x.
    TEST_ASSERT_EQUAL_UINT32(17u, (2u * 20u * cap) / 255u);
    TEST_ASSERT_EQUAL_UINT32(6u, (2u * 20u * lights::renderedDuty(255, cap)) / 255u);
}

void test_failsafe_hazard_overrides_everything() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = upState();
    s.braking = true; // would normally light the brake
    s.drsOpen = true; // would normally light the green tell
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

// --- DRS-open tell (vision decision 16) -------------------------------------
//
// While board #1's arbitrated drsOpen bit is set, the two OUTERMOST pixels of
// the rear brake bar glow steady green (the flap is in the rear wing; green
// is the TV-graphics DRS color); the bar's middle keeps the dim tail.

void test_drs_open_lights_green_bar_edges() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = upState();
    s.drsOpen = true;
    r.render(s, light_status::Up, kIgnOff, 0, px);

    const Rgb first = px[cfg.brake.start];
    const Rgb last = px[cfg.brake.start + cfg.brake.len - 1];
    TEST_ASSERT_TRUE(first.g > 0 && first.r == 0 && first.b == 0); // pure green
    TEST_ASSERT_TRUE(last.g > 0 && last.r == 0 && last.b == 0);
    // Middle of the bar keeps the dim red tail (the tell is edges-only).
    const Rgb mid = px[cfg.brake.start + 2];
    TEST_ASSERT_TRUE(mid.r > 0 && mid.g == 0);

    // DRS closed: no green anywhere on the bar.
    s.drsOpen = false;
    r.render(s, light_status::Up, kIgnOff, 0, px);
    for (uint8_t i = 0; i < cfg.brake.len; ++i) {
        TEST_ASSERT_TRUE(px[cfg.brake.start + i].g == 0);
    }
}

void test_drs_tell_never_masks_brake_light() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = upState();
    s.drsOpen = true;
    s.braking = true;
    r.render(s, light_status::Up, kIgnOff, 0, px);

    // Braking wins the whole bar: every pixel bright red, zero green.
    for (uint8_t i = 0; i < cfg.brake.len; ++i) {
        const Rgb p = px[cfg.brake.start + i];
        TEST_ASSERT_TRUE(p.r > 0 && p.g == 0 && p.b == 0);
    }
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

    // Return under the off threshold INSIDE the minimum-on window
    // (correctness-1): the latch HOLDS, so it is still lit at an on-phase.
    s.steeringPercent = 10;
    r.render(s, light_status::Up, kIgnOff, 300, px); // 300 % 660 < 330 -> lit half
    TEST_ASSERT_TRUE(!(segFirst(px, cfg.rightIndicator) == (Rgb{0, 0, 0})));

    // Past the window, sampled at an on-phase where a still-latched indicator
    // WOULD be lit: it self-cancels.
    r.render(s, light_status::Up, kIgnOff, 700, px); // elapsed 700 >= 660; 700 % 660 = 40
    TEST_ASSERT_TRUE(segFirst(px, cfg.rightIndicator) == (Rgb{0, 0, 0}));
}

namespace {

// Drives a `flickMs` steering flick starting at `startMs` and returns how many
// 1 ms samples inside the minimum-on window show the indicator lit. A fresh
// renderer per call: the latch state is the thing under test.
uint32_t litMsDuringFlick(int8_t steerPeak, const lights::Segment& seg, uint32_t startMs,
                          uint32_t flickMs) {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = upState();
    uint32_t lit = 0;
    for (uint32_t t = startMs; t < startMs + cfg.indicatorPeriodMs; ++t) {
        s.steeringPercent = (t < startMs + flickMs) ? steerPeak : 0;
        r.render(s, light_status::Up, kIgnOff, t, px);
        if (!(segFirst(px, seg) == (Rgb{0, 0, 0}))) ++lit;
    }
    return lit;
}

// A single-sample flick at `startMs` (enough to cross the on-threshold and
// latch -- the latch then holds regardless of steering for a full
// indicatorPeriodMs from that rising edge), scanning the WHOLE minimum-on
// window and returning the longest run of CONSECUTIVE 1 ms lit samples --
// the number the "shortest guaranteed continuous flash" comment claims,
// checked against the real renderer rather than a model of it.
uint32_t longestLitRunDuringFlick(int8_t steerPeak, const lights::Segment& seg,
                                   uint32_t startMs) {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = upState();
    uint32_t best = 0;
    uint32_t run = 0;
    for (uint32_t t = startMs; t < startMs + cfg.indicatorPeriodMs; ++t) {
        s.steeringPercent = (t == startMs) ? steerPeak : 0;
        r.render(s, light_status::Up, kIgnOff, t, px);
        if (!(segFirst(px, seg) == (Rgb{0, 0, 0}))) {
            ++run;
            if (run > best) best = run;
        } else {
            run = 0;
        }
    }
    return best;
}

} // namespace

// correctness-1: LightRenderer.hpp promised "minimum-on so a flick still
// completes one blink" and only hysteresis was implemented. A 240 ms flick
// landing in the blink's dark half latched on and cancelled again without
// lighting a single pixel -- invisible, on the cue booklet :172 sells as
// "blinkers that follow the steering".
//
// Checked at every interesting phase offset, both sides (the sim feeder's
// triangle() never goes negative, so the LEFT indicator had neither a demo
// nor a test before this): whatever the phase, the minimum-on window (one
// indicatorPeriodMs) always contains indicatorPeriodMs/2 of TOTAL lit time
// (minus the 1 ms the sampling grid can clip). That total can land in two
// separate runs rather than one contiguous half-cycle --
// test_indicator_minimum_on_worst_case_contiguous_run below pins the
// shortest guaranteed CONTINUOUS flash instead (indicatorPeriodMs/4).
void test_indicator_flick_still_completes_one_blink() {
    const uint32_t halfCycle = cfg.indicatorPeriodMs / 2u;
    const uint32_t kStarts[] = {0, 100, 329, 330, 500, 659};
    for (uint32_t start : kStarts) {
        const uint32_t litRight = litMsDuringFlick(100, cfg.rightIndicator, start, 240);
        TEST_ASSERT_GREATER_OR_EQUAL_UINT32(halfCycle - 1u, litRight);
        const uint32_t litLeft = litMsDuringFlick(-100, cfg.leftIndicator, start, 240);
        TEST_ASSERT_GREATER_OR_EQUAL_UINT32(halfCycle - 1u, litLeft);
    }

    // Without the minimum-on the same flick is invisible: 240 ms starting at
    // phase 330 sits entirely inside the dark half. Pinned as arithmetic so
    // the test cannot quietly become a tautology.
    TEST_ASSERT_TRUE(330u + 240u <= cfg.indicatorPeriodMs);

    // And the latch really does let go afterwards -- min-on defers the
    // self-cancel, it does not remove it.
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = upState();
    s.steeringPercent = 100;
    r.render(s, light_status::Up, kIgnOff, 330, px);
    s.steeringPercent = 0;
    r.render(s, light_status::Up, kIgnOff, 330 + cfg.indicatorPeriodMs, px); // cancels here
    r.render(s, light_status::Up, kIgnOff, 1320, px);                        // 1320 % 660 = 0
    TEST_ASSERT_TRUE(segFirst(px, cfg.rightIndicator) == (Rgb{0, 0, 0}));
}

// Pins the sentence review finding sl:correctness-1/timing corrected: the
// minimum-on window guarantees indicatorPeriodMs/2 of TOTAL lit time, but
// the shortest guaranteed CONTINUOUS flash is only indicatorPeriodMs/4 (a
// window starting mid-on-phase splits the lit time into two runs either
// side of a full dark half-cycle). Sweep every start phase against the real
// renderer: the bound must hold everywhere, AND be tight -- some phase must
// actually hit exactly indicatorPeriodMs/4, or "the shortest guaranteed
// flash is 165 ms" would be a safe-but-loose estimate rather than the true
// worst case.
void test_indicator_minimum_on_worst_case_contiguous_run_is_quarter_period() {
    const uint32_t period = cfg.indicatorPeriodMs;
    const uint32_t quarterPeriod = period / 4u;
    uint32_t worst = period; // shrinks to the true minimum below
    for (uint32_t start = 0; start < period; ++start) {
        const uint32_t runRight = longestLitRunDuringFlick(100, cfg.rightIndicator, start);
        TEST_ASSERT_GREATER_OR_EQUAL_UINT32(quarterPeriod, runRight);
        if (runRight < worst) worst = runRight;
    }
    TEST_ASSERT_EQUAL_UINT32(quarterPeriod, worst);
}

// The minimum-on window must not out-rank a deliberate opposite lock: a
// driver who steers hard the other way inside the window gets the other
// indicator at once, never both (which would read as hazard).
void test_opposite_lock_swaps_sides_inside_the_minimum_on_window() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = upState();

    s.steeringPercent = 100;
    r.render(s, light_status::Up, kIgnOff, 0, px);
    TEST_ASSERT_TRUE(!(segFirst(px, cfg.rightIndicator) == (Rgb{0, 0, 0})));

    s.steeringPercent = -100; // 100 ms later, well inside the 660 ms window
    r.render(s, light_status::Up, kIgnOff, 100, px);
    TEST_ASSERT_TRUE(!(segFirst(px, cfg.leftIndicator) == (Rgb{0, 0, 0})));
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

// --- sl:correctness-4: the composite order matches the declared order -------
//
// LightRenderer.hpp declares base -> DRS -> brake/rain/indicators ->
// low-battery -> hazard. The low-battery block used to run BEFORE the
// functional layer, so the declared priority was inverted. With the shipped
// segments nothing overlaps and nothing rendered differently -- which is
// exactly why it went unnoticed -- so the proof has to use an OVERLAPPING
// segment config, and the segments are an explicit bench tunable.

void test_low_battery_outranks_the_functional_layer_when_segments_overlap() {
    // A time where the indicator blink is ON (314 % 660 < 330) and the
    // low-battery triangle is part-way up (tri 314 -> level 100), so the
    // pulse color is distinguishable from both amber and the brake red.
    const uint32_t t = 314;

    // 1. Halo widened over the LEFT indicator: the alert wins the pixel, and
    // "wins" is visible as the absence of amber's green channel.
    {
        LightConfig c;
        c.halo = lights::Segment{22, 4}; // exactly the left indicator
        TEST_ASSERT_TRUE(c.valid());
        LightRenderer r(c);
        Rgb px[kNumPixels];
        VehicleState s = upState();
        s.steeringPercent = -100; // left indicator latched on
        s.lowBattery = true;
        r.render(s, light_status::Up, kIgnOff, t, px);
        const Rgb shared = px[c.leftIndicator.start];
        TEST_ASSERT_EQUAL_UINT8(0, shared.g); // amber would have g > 0
        TEST_ASSERT_EQUAL_UINT8(0, shared.b);
        TEST_ASSERT_TRUE(shared.r > 0); // and it is lit, not blanked
    }

    // 2. Halo widened over the BRAKE bar: the pulse owns it at its own
    // level, not the brake light's full red.
    {
        LightConfig c;
        c.halo = lights::Segment{0, 6}; // exactly the brake bar
        TEST_ASSERT_TRUE(c.valid());
        LightRenderer r(c);
        Rgb px[kNumPixels];
        VehicleState s = upState();
        s.braking = true;
        s.lowBattery = true;
        r.render(s, light_status::Up, kIgnOff, t, px);
        const Rgb shared = px[c.brake.start];
        TEST_ASSERT_EQUAL_UINT8(lights::renderedDuty(100, c.maxBrightness), shared.r);
        TEST_ASSERT_TRUE(shared.r < lights::renderedDuty(255, c.maxBrightness));
    }

    // 3. ...and the failsafe hazard still outranks the alert, unchanged.
    {
        LightConfig c;
        c.halo = lights::Segment{0, 30};
        LightRenderer r(c);
        Rgb px[kNumPixels];
        VehicleState s = upState();
        s.lowBattery = true;
        s.failsafe = true;
        r.render(s, light_status::Up, kIgnOff, 0, px);
        TEST_ASSERT_TRUE(allAmber(px));
    }
}

// The default segments really are disjoint from the halo, which is why the
// inversion was invisible -- pin that so the test above cannot silently
// become the only thing standing between a bench tune and a masked alert.
void test_default_segments_do_not_overlap_the_halo() {
    const LightConfig c;
    const lights::Segment functional[] = {c.brake, c.rainLight, c.leftIndicator,
                                          c.rightIndicator};
    for (const lights::Segment& seg : functional) {
        const uint16_t segEnd = static_cast<uint16_t>(seg.start) + seg.len;
        const uint16_t haloEnd = static_cast<uint16_t>(c.halo.start) + c.halo.len;
        TEST_ASSERT_TRUE(segEnd <= c.halo.start || seg.start >= haloEnd);
    }
}

// --- Showcase halo (owner decision D6) ---------------------------------------

namespace {

// Effective showcase frame as the monitor delivers it on a live link: bit0
// set, armed 0, no failsafe (the monitor zeroes the bit in NeverConnected
// and Lost, so this combination is the ONLY way the showcase look renders).
VehicleState showcaseUp() {
    VehicleState s;
    s.showcase = true;
    s.armed = false;
    s.failsafe = false;
    return s;
}

} // namespace

// The D6 base look: slow pure-teal breathe on the halo (never dark -- the
// floor keeps it alive at the dip), dim red tail lit, no amber signature
// anywhere, and it actually breathes.
void test_showcase_halo_breathes_teal_with_tail_lit() {
    LightRenderer r;
    Rgb px[kNumPixels];
    const VehicleState s = showcaseUp();

    uint8_t minG = 255;
    uint8_t maxG = 0;
    for (uint32_t t = 0; t <= 3000; t += 50) {
        r.render(s, light_status::Up, kIgnRunning, t, px);
        const Rgb halo = px[cfg.halo.start];
        TEST_ASSERT_EQUAL_UINT8(0, halo.r);         // pure teal family, never a red tinge
        TEST_ASSERT_TRUE(halo.g > 0 || halo.b > 0); // the floor: never fully dark
        if (halo.g < minG) minG = halo.g;
        if (halo.g > maxG) maxG = halo.g;
        const Rgb tail = segFirst(px, cfg.brake);
        TEST_ASSERT_TRUE(tail.r > 0); // dim red tail lit (base layer)
        TEST_ASSERT_EQUAL_UINT8(0, tail.g);
        TEST_ASSERT_TRUE(noAmber(px)); // showcase base never shows the hazard signature
    }
    TEST_ASSERT_TRUE(maxG > minG); // it breathes rather than holding a level
}

// WHY-comment made executable: grace and showcase mean OPPOSITE things
// (grace = PRE-FIRST-FRAME benefit of the doubt; showcase = frames PRESENT
// with modeFlags bit0), so the looks must be tell-apart-able at a glance.
// Pinned on three visible axes: tail lit vs off, pure-teal vs red-tinged
// peak, floor vs dips-to-black -- plus the different periods.
void test_showcase_breathe_distinct_from_grace_breathe() {
    LightRenderer rShow;
    LightRenderer rGrace;
    Rgb show[kNumPixels];
    Rgb grace[kNumPixels];
    const VehicleState s = showcaseUp();
    const VehicleState defaults; // NeverConnected effective state

    // Whole grace window (it escalates at 5000): tail axis on every sample.
    for (uint32_t t = 0; t <= 4750; t += 250) {
        rShow.render(s, light_status::Up, kIgnRunning, t, show);
        rGrace.render(defaults, light_status::NeverConnected, kIgnOff, t, grace);
        TEST_ASSERT_EQUAL_UINT8(0, show[cfg.halo.start].r); // showcase: never red-tinged
        TEST_ASSERT_TRUE(segFirst(show, cfg.brake).r > 0);  // showcase: tail lit
        TEST_ASSERT_TRUE(segFirst(grace, cfg.brake) == (Rgb{0, 0, 0})); // grace: halo-only
    }

    // Color-family axis at the grace peak (t = 1000): grace shows its
    // cyan-white tinge (r > 0), showcase stays pure teal (r == 0).
    rGrace.render(defaults, light_status::NeverConnected, kIgnOff, 1000, grace);
    rShow.render(s, light_status::Up, kIgnRunning, 1000, show);
    TEST_ASSERT_TRUE(grace[cfg.halo.start].r > 0);
    TEST_ASSERT_EQUAL_UINT8(0, show[cfg.halo.start].r);

    // Floor axis at a grace dark instant (t = 2000, its triangle zero):
    // grace halo is black, the showcase halo is still visibly lit.
    rGrace.render(defaults, light_status::NeverConnected, kIgnOff, 2000, grace);
    rShow.render(s, light_status::Up, kIgnRunning, 2000, show);
    TEST_ASSERT_TRUE(grace[cfg.halo.start] == (Rgb{0, 0, 0}));
    const Rgb showAt2000 = show[cfg.halo.start];
    TEST_ASSERT_TRUE(showAt2000.g > 0 || showAt2000.b > 0);

    // Period axis: the showcase level differs 2000 ms apart (a grace-period
    // clone would alias); 3000 ms apart it repeats exactly.
    rShow.render(s, light_status::Up, kIgnRunning, 3000, show);
    const Rgb showAt3000 = show[cfg.halo.start];
    rShow.render(s, light_status::Up, kIgnRunning, 1000, show);
    const Rgb showAt1000 = show[cfg.halo.start];
    TEST_ASSERT_FALSE(showAt1000 == showAt3000); // 2000 ms apart: different
    rShow.render(s, light_status::Up, kIgnRunning, 4000, show);
    TEST_ASSERT_TRUE(show[cfg.halo.start] == showAt1000); // 3000 ms apart: same
}

// --- sl:safety-1 (3): the minimum-rendered-duty floor ------------------------
//
// The defect this pins: with the cap applied before gamma, the quiet palette
// entries rendered at PWM 1 and the two assertions that existed
// (anyNonBlack / "not amber") both passed happily on a strip nobody could
// see. Every state whose JOB is to be seen while quiet is checked here
// through the real renderer, in rendered duty, at the shipped config.
//
// The absolute number is a floor, not a verdict: whether 6/255 reads in
// daylight is a bench judgement ([bench-TBD]; open_questions.md #55).

namespace {

uint8_t maxChannel(Rgb c) {
    const uint8_t rg = c.r > c.g ? c.r : c.g;
    return rg > c.b ? rg : c.b;
}

} // namespace

void test_every_designed_visible_state_clears_the_minimum_duty() {
    const uint8_t kMin = lights::kMinVisibleDuty;
    Rgb px[kNumPixels];

    // 1. Disarmed on a live link: dim-white halo AND the dim red tail.
    {
        LightRenderer r;
        VehicleState disarmed;
        disarmed.armed = false;
        disarmed.failsafe = false;
        r.render(disarmed, light_status::Up, kIgnOff, 0, px);
        TEST_ASSERT_GREATER_OR_EQUAL_UINT8(kMin, maxChannel(px[cfg.halo.start]));
        TEST_ASSERT_GREATER_OR_EQUAL_UINT8(kMin, maxChannel(segFirst(px, cfg.brake)));
    }

    // 2. Armed halo teal (the state the disarmed one must stay below).
    {
        LightRenderer r;
        r.render(upState(), light_status::Up, kIgnOff, 0, px);
        TEST_ASSERT_GREATER_OR_EQUAL_UINT8(kMin, maxChannel(px[cfg.halo.start]));
    }

    // 3. NeverConnected grace breathe AT ITS PEAK (it dips to black by
    // design -- that is the axis that tells it from the showcase breathe).
    {
        LightRenderer r;
        VehicleState defaults;
        r.render(defaults, light_status::NeverConnected, kIgnOff, 0, px); // seed
        uint8_t peak = 0;
        for (uint32_t t = 0; t <= 2000; t += 25) {
            r.render(defaults, light_status::NeverConnected, kIgnOff, t, px);
            const uint8_t m = maxChannel(px[cfg.halo.start]);
            if (m > peak) peak = m;
        }
        TEST_ASSERT_GREATER_OR_EQUAL_UINT8(kMin, peak);
    }

    // 4. Showcase breathe AT ITS FLOOR: this one never dips dark, so the
    // MINIMUM over a whole cycle is what must clear the bar -- and the tail
    // stays lit under it.
    {
        LightRenderer r;
        const VehicleState s = showcaseUp();
        uint8_t floorSeen = 255;
        for (uint32_t t = 0; t <= 3000; t += 25) {
            r.render(s, light_status::Up, kIgnRunning, t, px);
            const uint8_t m = maxChannel(px[cfg.halo.start]);
            if (m < floorSeen) floorSeen = m;
            TEST_ASSERT_GREATER_OR_EQUAL_UINT8(kMin, maxChannel(segFirst(px, cfg.brake)));
        }
        TEST_ASSERT_GREATER_OR_EQUAL_UINT8(kMin, floorSeen);
    }

    // 5. Failsafe hazard at an on-phase: every pixel amber, all of them lit.
    {
        LightRenderer r;
        VehicleState fs = upState();
        fs.failsafe = true;
        r.render(fs, light_status::Up, kIgnOff, 0, px); // 0 % 500 < 250 -> on
        for (uint8_t i = 0; i < kNumPixels; ++i) {
            TEST_ASSERT_GREATER_OR_EQUAL_UINT8(kMin, maxChannel(px[i]));
        }
    }

    // 6. Low-battery pulse at its peak (period/2 -> triangle at full).
    {
        LightRenderer r;
        VehicleState low = upState();
        low.lowBattery = true;
        r.render(low, light_status::Up, kIgnOff, cfg.lowBatteryPeriodMs / 2, px);
        TEST_ASSERT_GREATER_OR_EQUAL_UINT8(kMin, maxChannel(px[cfg.halo.start]));
    }

    // 7. The loud functional states, for completeness: brake, indicators,
    // rain light and the DRS tell all sit at the top of the range.
    {
        LightRenderer r;
        VehicleState s = upState();
        s.braking = true;
        s.drsOpen = true;
        s.steeringPercent = 100;
        r.render(s, light_status::Up, kIgnOff, 0, px);
        TEST_ASSERT_GREATER_OR_EQUAL_UINT8(kMin, maxChannel(segFirst(px, cfg.brake)));
        TEST_ASSERT_GREATER_OR_EQUAL_UINT8(kMin, maxChannel(segFirst(px, cfg.rightIndicator)));
    }
}

// The floor is a real constraint, not a tautology: at the pre-fix palette
// value (40) the disarmed halo rendered at PWM 1, which is what this whole
// finding was about. Pin the arithmetic so nobody "simplifies" the palette
// back.
void test_the_old_dim_value_would_fail_the_minimum() {
    TEST_ASSERT_EQUAL_UINT8(1, lights::renderedDuty(40, LightConfig{}.maxBrightness));
    TEST_ASSERT_TRUE(lights::renderedDuty(40, LightConfig{}.maxBrightness) <
                     lights::kMinVisibleDuty);
    // ...and the shipped values clear it by construction.
    TEST_ASSERT_GREATER_OR_EQUAL_UINT8(lights::kMinVisibleDuty,
                                       lights::renderedDuty(105, LightConfig{}.maxBrightness));
}

// Faults outrank the show, renderer-level (defense in depth under the
// monitor's own zero-on-Lost projection): a Lost link hazards even if a
// stale frame still says showcase, and a frame-level failsafe (D4 row 3:
// the radio died mid-showcase) hazards on a live link.
void test_showcase_lost_or_failsafe_is_hazard_not_show() {
    LightRenderer r;
    Rgb px[kNumPixels];
    r.render(showcaseUp(), light_status::Lost, kIgnOff, 0, px);
    TEST_ASSERT_TRUE(allAmber(px));

    LightRenderer r2;
    VehicleState fs = showcaseUp();
    fs.failsafe = true;
    r2.render(fs, light_status::Up, kIgnOff, 0, px);
    TEST_ASSERT_TRUE(allAmber(px));
}

// D5, the lights half: low battery ENDS the show -- the red pulse OWNS the
// halo (overwrite, not blend: at the pulse's dark phase the halo is black,
// the breathe must not peek through), while the tail keeps its base red.
// The engine half (ignition Off => silence) is pinned in test_enginesim;
// kIgnOff here mirrors that state.
void test_showcase_low_battery_pulse_wins() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = showcaseUp();
    s.lowBattery = true;

    // Pulse peak: halo pure red, no teal anywhere on it.
    r.render(s, light_status::Up, kIgnOff, cfg.lowBatteryPeriodMs / 2, px);
    for (uint8_t i = 0; i < cfg.halo.len; ++i) {
        const Rgb p = px[cfg.halo.start + i];
        TEST_ASSERT_TRUE(p.r > 0);
        TEST_ASSERT_EQUAL_UINT8(0, p.g);
        TEST_ASSERT_EQUAL_UINT8(0, p.b);
    }

    // Pulse dark phase: the halo is BLACK -- the showcase breathe (whose
    // floor would keep it lit) has been overwritten, not mixed.
    r.render(s, light_status::Up, kIgnOff, 0, px);
    for (uint8_t i = 0; i < cfg.halo.len; ++i) {
        TEST_ASSERT_TRUE(px[cfg.halo.start + i] == (Rgb{0, 0, 0}));
    }
}

// Functional layers keep outranking the showcase base: brake overwrites the
// tail (layer-order pin -- a truthful showcase frame never brakes, but the
// compositor must not care) and the steering indicators blink amber over a
// live table-demo link (D9: steering IS live in showcase).
void test_showcase_brake_and_indicators_still_outrank_base() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = showcaseUp();
    s.braking = true;
    s.steeringPercent = -80;

    r.render(s, light_status::Up, kIgnRunning, 100, px); // indicator on-phase
    const Rgb brake = segFirst(px, cfg.brake);
    TEST_ASSERT_TRUE(brake.r > 20); // bright red, not the dim tail
    TEST_ASSERT_EQUAL_UINT8(0, brake.g);
    const Rgb left = segFirst(px, cfg.leftIndicator);
    TEST_ASSERT_TRUE(left.r > 0 && left.g > 0 && left.b == 0); // amber blink
    TEST_ASSERT_EQUAL_UINT8(0, px[cfg.halo.start].r); // halo still the show
}

// Scene-1 opening beats ride the existing ignition animation for free: the
// starter comet sweeps while Cranking, and the catch flash crossfades INTO
// THE BREATHE (no snap to armed teal -- the blend target is the showcase
// base). The comet/flash cyan is far brighter than the breathe ever gets,
// so the two phases are unmistakable.
void test_showcase_crank_comet_and_catch_flash_settle_into_breathe() {
    LightRenderer r;
    Rgb px[kNumPixels];
    const VehicleState s = showcaseUp();

    // Cranking: the cyan comet head stands well above the breathe's range.
    r.render(s, light_status::Up, kIgnCranking, 40, px);
    uint8_t brightest = 0;
    for (uint8_t i = 0; i < cfg.halo.len; ++i) {
        if (px[cfg.halo.start + i].g > brightest) brightest = px[cfg.halo.start + i].g;
    }
    TEST_ASSERT_TRUE(brightest > 10); // breathe tops out well under this

    // Catch: flash starts on the Running edge, bright cyan...
    r.render(s, light_status::Up, kIgnRunning, 1000, px);
    const Rgb atCatch = px[cfg.halo.start];
    TEST_ASSERT_TRUE(atCatch.g > 10);
    // ...and at flash end the halo IS the breathe for that instant.
    r.render(s, light_status::Up, kIgnRunning, 1000 + cfg.ignitionFlashMs, px);
    const Rgb settled = px[cfg.halo.start];
    TEST_ASSERT_EQUAL_UINT8(0, settled.r);
    TEST_ASSERT_TRUE(settled.g > 0);
    TEST_ASSERT_TRUE(settled.g < atCatch.g); // gentle show, not the flash
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_config_valid_and_within_power_budget);
    RUN_TEST(test_gamma_lut_matches_the_libm_reference);
    RUN_TEST(test_cap_is_applied_before_gamma_not_after);
    RUN_TEST(test_failsafe_hazard_overrides_everything);
    RUN_TEST(test_link_lost_forces_hazard_even_if_frame_not_failsafe);
    RUN_TEST(test_never_connected_is_calm_not_hazard);
    RUN_TEST(test_never_connected_escalates_to_hazard_after_grace);
    RUN_TEST(test_never_connected_grace_measured_from_first_render);
    RUN_TEST(test_never_connected_grace_validation_bounds);
    RUN_TEST(test_brake_lights_on_braking);
    RUN_TEST(test_drs_open_lights_green_bar_edges);
    RUN_TEST(test_drs_tell_never_masks_brake_light);
    RUN_TEST(test_indicator_hysteresis_and_selfcancel);
    RUN_TEST(test_indicator_flick_still_completes_one_blink);
    RUN_TEST(test_indicator_minimum_on_worst_case_contiguous_run_is_quarter_period);
    RUN_TEST(test_opposite_lock_swaps_sides_inside_the_minimum_on_window);
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
    RUN_TEST(test_low_battery_outranks_the_functional_layer_when_segments_overlap);
    RUN_TEST(test_default_segments_do_not_overlap_the_halo);
    RUN_TEST(test_showcase_halo_breathes_teal_with_tail_lit);
    RUN_TEST(test_showcase_breathe_distinct_from_grace_breathe);
    RUN_TEST(test_every_designed_visible_state_clears_the_minimum_duty);
    RUN_TEST(test_the_old_dim_value_would_fail_the_minimum);
    RUN_TEST(test_showcase_lost_or_failsafe_is_hazard_not_show);
    RUN_TEST(test_showcase_low_battery_pulse_wins);
    RUN_TEST(test_showcase_brake_and_indicators_still_outrank_base);
    RUN_TEST(test_showcase_crank_comet_and_catch_flash_settle_into_breathe);
    return UNITY_END();
}
