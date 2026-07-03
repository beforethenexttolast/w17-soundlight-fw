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
    r.render(s, light_status::Up, /*nowMs=*/0, px);
    for (uint8_t i = 0; i < kNumPixels; ++i) {
        TEST_ASSERT_TRUE(px[i].r > 0 && px[i].g > 0 && px[i].b == 0);
    }
    // Half a period later: all off.
    r.render(s, light_status::Up, cfg.hazardPeriodMs / 2, px);
    for (uint8_t i = 0; i < kNumPixels; ++i) {
        TEST_ASSERT_TRUE(px[i] == (Rgb{0, 0, 0}));
    }
}

void test_link_lost_forces_hazard_even_if_frame_not_failsafe() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = upState(); // frame says fine...
    r.render(s, light_status::Lost, 0, px); // ...but the link is Lost
    for (uint8_t i = 0; i < kNumPixels; ++i) {
        TEST_ASSERT_TRUE(px[i].r > 0 && px[i].g > 0 && px[i].b == 0);
    }
}

void test_never_connected_is_calm_not_hazard() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s; // defaults
    r.render(s, light_status::NeverConnected, 500, px);
    // Waiting breathe is teal-ish on the halo, never the amber hazard.
    bool anyAmber = false;
    for (uint8_t i = 0; i < kNumPixels; ++i) {
        if (px[i].r > 0 && px[i].g > 0 && px[i].b == 0 && px[i].r > 20) anyAmber = true;
    }
    TEST_ASSERT_FALSE(anyAmber);
}

void test_brake_lights_on_braking() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = upState();
    s.braking = true;
    r.render(s, light_status::Up, 0, px);
    const Rgb brake = segFirst(px, cfg.brake);
    TEST_ASSERT_TRUE(brake.r > brake.g && brake.r > brake.b); // dominant red
}

void test_indicator_hysteresis_and_selfcancel() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = upState();

    // Steer hard right: right indicator active (sample at an on-phase).
    s.steeringPercent = 60;
    r.render(s, light_status::Up, 0, px);
    TEST_ASSERT_TRUE(!(segFirst(px, cfg.rightIndicator) == (Rgb{0, 0, 0})));

    // Ease to +30 (between off=20 and on=40): hysteresis holds it ON.
    s.steeringPercent = 30;
    r.render(s, light_status::Up, 0, px);
    TEST_ASSERT_TRUE(!(segFirst(px, cfg.rightIndicator) == (Rgb{0, 0, 0})));

    // Return under the off threshold: self-cancels.
    s.steeringPercent = 10;
    r.render(s, light_status::Up, 0, px);
    TEST_ASSERT_TRUE(segFirst(px, cfg.rightIndicator) == (Rgb{0, 0, 0}));
}

void test_rain_light_flashes_only_while_harvesting_in_ers_mode() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = upState();
    s.driveMode = 2;
    s.ersPercent = 40;

    // Seed the harvest detector (first frame establishes baseline, no rise).
    r.render(s, light_status::Up, 0, px);

    // ersPercent rises -> harvesting -> rain light flashes (sample on-phase).
    s.ersPercent = 42;
    r.render(s, light_status::Up, 100, px); // 100 % 250 < 125 => on
    TEST_ASSERT_TRUE(!(segFirst(px, cfg.rainLight) == (Rgb{0, 0, 0})));

    // No rise for a while -> harvest window closes -> rain light off.
    r.render(s, light_status::Up, 100 + 500, px);
    TEST_ASSERT_TRUE(segFirst(px, cfg.rainLight) == (Rgb{0, 0, 0}));
}

void test_rain_light_ignores_deploy_only() {
    LightRenderer r;
    Rgb px[kNumPixels];
    VehicleState s = upState();
    s.driveMode = 2;
    s.ersPercent = 80;
    r.render(s, light_status::Up, 0, px);

    // Deploying: ersPercent FALLS -> not harvesting -> rain light stays off.
    s.ersDeploying = true;
    s.ersPercent = 76;
    r.render(s, light_status::Up, 100, px);
    TEST_ASSERT_TRUE(segFirst(px, cfg.rainLight) == (Rgb{0, 0, 0}));
}

void test_halo_teal_armed_dim_when_disarmed() {
    LightRenderer r;
    Rgb px[kNumPixels];

    VehicleState armed = upState();
    r.render(armed, light_status::Up, 0, px);
    const Rgb haloArmed = segFirst(px, cfg.halo);
    TEST_ASSERT_TRUE(haloArmed.g >= haloArmed.r); // teal: green/blue dominant

    VehicleState disarmed;
    disarmed.armed = false;
    disarmed.failsafe = false;
    // Disarmed but link Up (board #1 can send disarmed-idle frames).
    r.render(disarmed, light_status::Up, 0, px);
    TEST_ASSERT_TRUE(anyNonBlack(px)); // halo dim-white, not black
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_config_valid_and_within_power_budget);
    RUN_TEST(test_failsafe_hazard_overrides_everything);
    RUN_TEST(test_link_lost_forces_hazard_even_if_frame_not_failsafe);
    RUN_TEST(test_never_connected_is_calm_not_hazard);
    RUN_TEST(test_brake_lights_on_braking);
    RUN_TEST(test_indicator_hysteresis_and_selfcancel);
    RUN_TEST(test_rain_light_flashes_only_while_harvesting_in_ers_mode);
    RUN_TEST(test_rain_light_ignores_deploy_only);
    RUN_TEST(test_halo_teal_armed_dim_when_disarmed);
    return UNITY_END();
}
