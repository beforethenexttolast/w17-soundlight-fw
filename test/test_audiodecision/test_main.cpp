#include <unity.h>

#include <cstdint>

// Focused tests for the pure production audio decisions in lib/audiodecision:
// the engine-state -> synth-volume mapping and the control-heartbeat dead-man.
// These call the SAME functions src/main.cpp uses; expected results are fixed
// literals, never recomputed from a copy of the production formula/boundary.

#include "audiodecision/AudioDecision.hpp"
#include "enginesim/EngineSim.hpp"

using audiodecision::AudioRuntimeAction;
using audiodecision::classifyWrite;
using audiodecision::isAudioHeartbeatStale;
using audiodecision::runtimeActionFor;
using audiodecision::synthVolumeFor;
using audiodecision::WriteOutcome;
using enginesim::Ignition;

// The production runtime write requests 1024 bytes (256 frames * 2 ch * 2 B).
// Fixed here as a literal so the classification tests never recompute it.
static constexpr size_t kRequestedBytes = 1024;

// A representative non-success esp_err_t. This is the verified numeric value of
// ESP_ERR_INVALID_ARG (0x102) in the installed Arduino-ESP32 framework, fixed
// as a plain literal so no ESP-IDF header is imported. The exact value is
// immaterial to the classifier: it treats EVERY nonzero status as a failure.
static constexpr int32_t kNonSuccessStatus = 0x102;

void setUp() {}
void tearDown() {}

// ---- Volume mapping -------------------------------------------------------

// Off is silent regardless of throttle.
void test_volume_off_is_silent() {
    TEST_ASSERT_EQUAL_UINT8(0, synthVolumeFor(Ignition::Off, 0));
    TEST_ASSERT_EQUAL_UINT8(0, synthVolumeFor(Ignition::Off, 100));
}

// Cranking is a fixed quiet level, throttle-independent.
void test_volume_cranking_is_fixed() {
    TEST_ASSERT_EQUAL_UINT8(70, synthVolumeFor(Ignition::Cranking, 0));
    TEST_ASSERT_EQUAL_UINT8(70, synthVolumeFor(Ignition::Cranking, 100));
}

// Running at zero throttle is the base volume.
void test_volume_running_min_throttle() {
    TEST_ASSERT_EQUAL_UINT8(90, synthVolumeFor(Ignition::Running, 0));
}

// Running at half throttle: 90 + 50*165/100 = 90 + 82 (8250/100 truncates) = 172.
void test_volume_running_half_throttle_integer_math() {
    TEST_ASSERT_EQUAL_UINT8(172, synthVolumeFor(Ignition::Running, 50));
}

// Running at max valid throttle saturates the byte at 255.
void test_volume_running_max_throttle() {
    TEST_ASSERT_EQUAL_UINT8(255, synthVolumeFor(Ignition::Running, 100));
}

// Odd throttles that expose truncating integer division:
//   throttle 1  -> 90 + 165/100   = 90 + 1   = 91
//   throttle 61 -> 90 + 10065/100 = 90 + 100 = 190
void test_volume_running_truncation_odd_throttles() {
    TEST_ASSERT_EQUAL_UINT8(91, synthVolumeFor(Ignition::Running, 1));
    TEST_ASSERT_EQUAL_UINT8(190, synthVolumeFor(Ignition::Running, 61));
}

// Any ignition value that is neither Off nor Cranking (here an unknown enum
// representation) falls through to the Running throttle-scaled path.
void test_volume_unknown_ignition_takes_running_path() {
    const Ignition unknown = static_cast<Ignition>(7);
    TEST_ASSERT_EQUAL_UINT8(90, synthVolumeFor(unknown, 0));
    TEST_ASSERT_EQUAL_UINT8(255, synthVolumeFor(unknown, 100));
}

// ---- Heartbeat dead-man (timeout 500 ms, strict greater-than) -------------

void test_deadman_fresh_within_timeout() {
    // elapsed 0 ms: heartbeat just stamped.
    TEST_ASSERT_FALSE(isAudioHeartbeatStale(1000u, 1000u, 500u));
    // elapsed 250 ms: comfortably fresh.
    TEST_ASSERT_FALSE(isAudioHeartbeatStale(1250u, 1000u, 500u));
}

void test_deadman_stale_past_timeout() {
    // elapsed 501 ms: just over the line.
    TEST_ASSERT_TRUE(isAudioHeartbeatStale(1501u, 1000u, 500u));
    // A much larger gap is also stale.
    TEST_ASSERT_TRUE(isAudioHeartbeatStale(5000000u, 1000u, 500u));
}

void test_deadman_boundary_499_500_501() {
    TEST_ASSERT_FALSE(isAudioHeartbeatStale(1499u, 1000u, 500u)); // 499 -> fresh
    TEST_ASSERT_FALSE(isAudioHeartbeatStale(1500u, 1000u, 500u)); // 500 -> fresh
    TEST_ASSERT_TRUE(isAudioHeartbeatStale(1501u, 1000u, 500u));  // 501 -> stale
}

// millis() is uint32_t and wraps at 2^32; the decision must use unsigned
// subtraction so a heartbeat stamped just before the wrap stays correct.
void test_deadman_uint32_wraparound() {
    // now wrapped past 0; heartbeat 16 ms earlier (elapsed 16) -> fresh.
    TEST_ASSERT_FALSE(isAudioHeartbeatStale(10u, UINT32_MAX - 5u, 500u));
    // exactly 500 ms elapsed across the wrap -> fresh.
    TEST_ASSERT_FALSE(isAudioHeartbeatStale(100u, UINT32_MAX - 399u, 500u));
    // 501 ms elapsed across the wrap -> stale.
    TEST_ASSERT_TRUE(isAudioHeartbeatStale(101u, UINT32_MAX - 399u, 500u));
}

void test_deadman_recovery() {
    // Gone stale (elapsed 1000 ms)...
    TEST_ASSERT_TRUE(isAudioHeartbeatStale(2000u, 1000u, 500u));
    // ...then a newer heartbeat lands and the decision reports fresh again.
    TEST_ASSERT_FALSE(isAudioHeartbeatStale(2000u, 2000u, 500u));
}

// ---- Runtime write classification (SLR-4) ---------------------------------
// classifyWrite: status dominates; only exact requested==written with a zero
// status is Complete. Expected outcomes are explicit enum literals.

// Zero status, full 1024 bytes queued -> Complete.
void test_classify_success_full_write_is_complete() {
    TEST_ASSERT_EQUAL_INT(static_cast<int>(WriteOutcome::Complete),
                          static_cast<int>(classifyWrite(0, kRequestedBytes, 1024)));
}

// Zero status but fewer/zero/more bytes than requested -> ShortWrite.
void test_classify_success_zero_bytes_is_short_write() {
    TEST_ASSERT_EQUAL_INT(static_cast<int>(WriteOutcome::ShortWrite),
                          static_cast<int>(classifyWrite(0, kRequestedBytes, 0)));
}

void test_classify_success_half_bytes_is_short_write() {
    TEST_ASSERT_EQUAL_INT(static_cast<int>(WriteOutcome::ShortWrite),
                          static_cast<int>(classifyWrite(0, kRequestedBytes, 512)));
}

void test_classify_success_over_bytes_is_short_write() {
    TEST_ASSERT_EQUAL_INT(static_cast<int>(WriteOutcome::ShortWrite),
                          static_cast<int>(classifyWrite(0, kRequestedBytes, 2048)));
}

// Non-success status -> DriverError regardless of byte count (status dominates),
// including when the byte count would otherwise look Complete or ShortWrite.
void test_classify_error_zero_bytes_is_driver_error() {
    TEST_ASSERT_EQUAL_INT(static_cast<int>(WriteOutcome::DriverError),
                          static_cast<int>(classifyWrite(kNonSuccessStatus, kRequestedBytes, 0)));
}

void test_classify_error_half_bytes_is_driver_error() {
    TEST_ASSERT_EQUAL_INT(static_cast<int>(WriteOutcome::DriverError),
                          static_cast<int>(classifyWrite(kNonSuccessStatus, kRequestedBytes, 512)));
}

void test_classify_error_full_bytes_is_driver_error() {
    TEST_ASSERT_EQUAL_INT(static_cast<int>(WriteOutcome::DriverError),
                          static_cast<int>(classifyWrite(kNonSuccessStatus, kRequestedBytes, 1024)));
}

// Defensive zero-request coverage (pure-function; production never writes 0 B):
// 0 requested / 0 written / success is an exact match -> Complete.
void test_classify_zero_request_zero_written_is_complete() {
    TEST_ASSERT_EQUAL_INT(static_cast<int>(WriteOutcome::Complete),
                          static_cast<int>(classifyWrite(0, 0, 0)));
}

// 0 requested but a nonzero count reported -> ShortWrite.
void test_classify_zero_request_nonzero_written_is_short_write() {
    TEST_ASSERT_EQUAL_INT(static_cast<int>(WriteOutcome::ShortWrite),
                          static_cast<int>(classifyWrite(0, 0, 4)));
}

// ---- Runtime action mapping (SLR-4) ---------------------------------------

void test_action_complete_continues() {
    TEST_ASSERT_EQUAL_INT(static_cast<int>(AudioRuntimeAction::Continue),
                          static_cast<int>(runtimeActionFor(WriteOutcome::Complete)));
}

void test_action_short_write_disables() {
    TEST_ASSERT_EQUAL_INT(static_cast<int>(AudioRuntimeAction::Disable),
                          static_cast<int>(runtimeActionFor(WriteOutcome::ShortWrite)));
}

void test_action_driver_error_disables() {
    TEST_ASSERT_EQUAL_INT(static_cast<int>(AudioRuntimeAction::Disable),
                          static_cast<int>(runtimeActionFor(WriteOutcome::DriverError)));
}

// ---- Policy properties (SLR-4) --------------------------------------------

// Every non-Complete outcome maps to Disable; Complete is the only Continue.
void test_policy_only_complete_continues() {
    TEST_ASSERT_EQUAL_INT(static_cast<int>(AudioRuntimeAction::Continue),
                          static_cast<int>(runtimeActionFor(WriteOutcome::Complete)));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(AudioRuntimeAction::Disable),
                          static_cast<int>(runtimeActionFor(WriteOutcome::ShortWrite)));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(AudioRuntimeAction::Disable),
                          static_cast<int>(runtimeActionFor(WriteOutcome::DriverError)));
}

// The decision is a pure function: identical inputs give identical results, and
// there is no hidden counter/backoff that changes the verdict on repetition.
void test_policy_repeated_inputs_are_stable() {
    for (int i = 0; i < 5; ++i) {
        TEST_ASSERT_EQUAL_INT(static_cast<int>(WriteOutcome::Complete),
                              static_cast<int>(classifyWrite(0, kRequestedBytes, 1024)));
        TEST_ASSERT_EQUAL_INT(static_cast<int>(WriteOutcome::DriverError),
                              static_cast<int>(classifyWrite(kNonSuccessStatus, kRequestedBytes, 1024)));
        TEST_ASSERT_EQUAL_INT(static_cast<int>(AudioRuntimeAction::Disable),
                              static_cast<int>(runtimeActionFor(WriteOutcome::DriverError)));
    }
}

// Status dominates the byte count: a non-success status with a byte count that
// would otherwise be Complete still classifies (and disables) as DriverError.
void test_policy_status_dominates_byte_count() {
    const WriteOutcome errWouldBeComplete = classifyWrite(kNonSuccessStatus, kRequestedBytes, 1024);
    const WriteOutcome okShort = classifyWrite(0, kRequestedBytes, 1024 - 4);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(WriteOutcome::DriverError),
                          static_cast<int>(errWouldBeComplete));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(AudioRuntimeAction::Disable),
                          static_cast<int>(runtimeActionFor(errWouldBeComplete)));
    // Contrast: same byte-mismatch but a success status is only a ShortWrite.
    TEST_ASSERT_EQUAL_INT(static_cast<int>(WriteOutcome::ShortWrite), static_cast<int>(okShort));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_volume_off_is_silent);
    RUN_TEST(test_volume_cranking_is_fixed);
    RUN_TEST(test_volume_running_min_throttle);
    RUN_TEST(test_volume_running_half_throttle_integer_math);
    RUN_TEST(test_volume_running_max_throttle);
    RUN_TEST(test_volume_running_truncation_odd_throttles);
    RUN_TEST(test_volume_unknown_ignition_takes_running_path);
    RUN_TEST(test_deadman_fresh_within_timeout);
    RUN_TEST(test_deadman_stale_past_timeout);
    RUN_TEST(test_deadman_boundary_499_500_501);
    RUN_TEST(test_deadman_uint32_wraparound);
    RUN_TEST(test_deadman_recovery);
    RUN_TEST(test_classify_success_full_write_is_complete);
    RUN_TEST(test_classify_success_zero_bytes_is_short_write);
    RUN_TEST(test_classify_success_half_bytes_is_short_write);
    RUN_TEST(test_classify_success_over_bytes_is_short_write);
    RUN_TEST(test_classify_error_zero_bytes_is_driver_error);
    RUN_TEST(test_classify_error_half_bytes_is_driver_error);
    RUN_TEST(test_classify_error_full_bytes_is_driver_error);
    RUN_TEST(test_classify_zero_request_zero_written_is_complete);
    RUN_TEST(test_classify_zero_request_nonzero_written_is_short_write);
    RUN_TEST(test_action_complete_continues);
    RUN_TEST(test_action_short_write_disables);
    RUN_TEST(test_action_driver_error_disables);
    RUN_TEST(test_policy_only_complete_continues);
    RUN_TEST(test_policy_repeated_inputs_are_stable);
    RUN_TEST(test_policy_status_dominates_byte_count);
    return UNITY_END();
}
