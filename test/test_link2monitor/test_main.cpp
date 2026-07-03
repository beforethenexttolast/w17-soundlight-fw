#include <unity.h>

#include "link2/Link2Codec.hpp"
#include "link2monitor/Link2Monitor.hpp"

using link2::VehicleState;
using link2monitor::Link2Monitor;
using link2monitor::LinkStatus;

namespace {

// Feeds one encoded frame built from `s` at time nowMs.
void feedFrame(Link2Monitor& mon, const VehicleState& s, uint32_t nowMs) {
    uint8_t frame[link2::kFrameLen];
    link2::encodeFrame(s, frame);
    for (uint8_t b : frame) {
        mon.feedByte(b, nowMs);
    }
}

VehicleState driving() {
    VehicleState s;
    s.throttlePercent = 80;
    s.steeringPercent = 30;
    s.braking = false;
    s.armed = true;
    s.failsafe = false;
    s.lowBattery = true; // latched judgment from board #1
    s.gear = 3;
    s.rpm = 4000;
    s.batteryMv = 7100;
    s.ersPercent = 55;
    s.driveMode = 2;
    return s;
}

} // namespace

void setUp() {}
void tearDown() {}

void test_boots_never_connected_and_failsafe() {
    Link2Monitor mon;
    TEST_ASSERT_EQUAL(LinkStatus::NeverConnected, mon.status());
    TEST_ASSERT_TRUE(mon.state().failsafe);
    TEST_ASSERT_EQUAL_INT8(0, mon.state().throttlePercent);

    mon.poll(100000); // time passing without a frame does not change this
    TEST_ASSERT_EQUAL(LinkStatus::NeverConnected, mon.status());
}

void test_goes_up_on_first_frame() {
    Link2Monitor mon;
    feedFrame(mon, driving(), 100);
    TEST_ASSERT_EQUAL(LinkStatus::Up, mon.status());
    TEST_ASSERT_EQUAL_INT8(80, mon.state().throttlePercent);
    TEST_ASSERT_EQUAL_UINT16(4000, mon.state().rpm);
    TEST_ASSERT_FALSE(mon.state().failsafe);
}

void test_staleness_flips_at_exactly_the_window() {
    Link2Monitor mon; // stalenessMs = 500
    feedFrame(mon, driving(), 100);

    mon.poll(599); // 499ms elapsed: still Up
    TEST_ASSERT_EQUAL(LinkStatus::Up, mon.status());

    mon.poll(600); // exactly 500ms: Lost (>= is inclusive)
    TEST_ASSERT_EQUAL(LinkStatus::Lost, mon.status());
}

void test_per_field_staleness_projection() {
    Link2Monitor mon;
    feedFrame(mon, driving(), 100);
    mon.poll(1000); // Lost

    const VehicleState& s = mon.state();
    // Commands zeroed + failsafe asserted.
    TEST_ASSERT_EQUAL_INT8(0, s.throttlePercent);
    TEST_ASSERT_EQUAL_INT8(0, s.steeringPercent);
    TEST_ASSERT_FALSE(s.armed);
    TEST_ASSERT_TRUE(s.failsafe);
    // Motion telemetry zeroed (stale rpm must not drive sound/lights).
    TEST_ASSERT_EQUAL_UINT16(0, s.rpm);
    // Latched / garnish fields held last-known.
    TEST_ASSERT_TRUE(s.lowBattery);
    TEST_ASSERT_EQUAL_UINT16(7100, s.batteryMv);
    TEST_ASSERT_EQUAL_UINT8(3, s.gear);       // holding gear avoids phantom shift blips
    TEST_ASSERT_EQUAL_UINT8(55, s.ersPercent);
    TEST_ASSERT_EQUAL_UINT8(2, s.driveMode);
}

void test_recovers_on_next_good_frame() {
    Link2Monitor mon;
    feedFrame(mon, driving(), 100);
    mon.poll(1000); // Lost
    TEST_ASSERT_EQUAL(LinkStatus::Lost, mon.status());

    VehicleState s2 = driving();
    s2.throttlePercent = 20;
    feedFrame(mon, s2, 1100);
    TEST_ASSERT_EQUAL(LinkStatus::Up, mon.status());
    TEST_ASSERT_EQUAL_INT8(20, mon.state().throttlePercent);
    TEST_ASSERT_FALSE(mon.state().failsafe);
}

void test_config_valid() {
    TEST_ASSERT_TRUE(link2monitor::Link2MonitorConfig{}.valid());
    link2monitor::Link2MonitorConfig bad;
    bad.stalenessMs = 0;
    TEST_ASSERT_FALSE(bad.valid());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_boots_never_connected_and_failsafe);
    RUN_TEST(test_goes_up_on_first_frame);
    RUN_TEST(test_staleness_flips_at_exactly_the_window);
    RUN_TEST(test_per_field_staleness_projection);
    RUN_TEST(test_recovers_on_next_good_frame);
    RUN_TEST(test_config_valid);
    return UNITY_END();
}
