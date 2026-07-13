#pragma once

#include <cstddef>
#include <cstdint>

#include "enginesim/EngineSim.hpp"

// Pure audio-task decisions, shared verbatim by src/main.cpp and the native
// tests so production constants and the dead-man boundary can never drift into
// a private test copy again (SL-1 / SL-N1). No hardware, no Arduino headers,
// no allocation -- integer-only and constexpr-evaluable.
namespace audiodecision {

// Requested synth volume for the current engine state: silent when Off, a
// quiet fixed level while Cranking, and rising with throttle while Running.
// This is the single production source of the mapping main.cpp packs into the
// cross-core synth-param word.
//
// Any ignition value other than Off/Cranking -- Running, or an unknown enum
// representation -- takes the throttle-scaled path, matching the original
// inline decision exactly. Integer promotion (uint8_t throttle -> int),
// truncating division, and the final uint8_t narrowing are preserved verbatim:
// at throttlePercent 100 the result saturates at 255.
constexpr uint8_t synthVolumeFor(enginesim::Ignition ignition, uint8_t throttlePercent) {
    if (ignition == enginesim::Ignition::Off) return 0;
    if (ignition == enginesim::Ignition::Cranking) return 70;
    return static_cast<uint8_t>(90 + throttlePercent * 165 / 100); // 90..255
}

// Audio-task dead-man: the control loop stamps a heartbeat (millis()) each
// tick; if none has arrived within timeoutMs the audio task forces silent
// params, so a wedged control loop can't leave the engine screaming forever.
//
// Unsigned subtraction so uint32_t millis() wraparound stays correct (never
// convert to a signed elapsed). The boundary is strictly greater-than:
// elapsed == timeoutMs is still fresh; elapsed == timeoutMs + 1 is stale.
constexpr bool isAudioHeartbeatStale(uint32_t nowMs, uint32_t heartbeatMs, uint32_t timeoutMs) {
    return nowMs - heartbeatMs > timeoutMs;
}

// ---- Runtime I2S write outcome (SLR-4) ------------------------------------
// Classification of a single completed i2s_write call, from the small POD the
// HAL returns (esp_err_t status + requested/written byte counts). Pure and
// integer-only: no ESP-IDF or Arduino headers, no dependency on the HAL type --
// main.cpp unpacks the HAL result and passes the three primitives in, so this
// decision stays testable natively.
enum class WriteOutcome : uint8_t {
    Complete,    // driver accepted the whole buffer
    ShortWrite,  // driver reported success but queued fewer bytes than asked
    DriverError, // driver returned a non-success status
};

// Status DOMINATES the byte-count interpretation: any non-success status is a
// DriverError no matter what byte count came back. `status` is the numeric
// esp_err_t; success is ESP_OK, which is numerically 0 in the installed
// Arduino-ESP32 framework (verified against tools/sdk/.../esp_err.h). Only an
// exact requested==written match with a zero status is Complete; a zero status
// with any other count is a ShortWrite.
constexpr WriteOutcome classifyWrite(int32_t status, size_t requestedBytes, size_t bytesWritten) {
    if (status != 0) return WriteOutcome::DriverError;
    if (bytesWritten != requestedBytes) return WriteOutcome::ShortWrite;
    return WriteOutcome::Complete;
}

// What the audio task should do after a write. This batch is deliberately
// terminal: the only non-Continue action is to permanently disable audio for
// the current boot (the task deletes itself). No retry, backoff, or recovery
// state exists here or is implied by the enum.
enum class AudioRuntimeAction : uint8_t {
    Continue, // keep pumping audio
    Disable,  // stop the audio task for the rest of this boot
};

// Every non-Complete outcome disables audio; Complete continues. Total over the
// enum so any outcome maps to exactly one action.
constexpr AudioRuntimeAction runtimeActionFor(WriteOutcome outcome) {
    return outcome == WriteOutcome::Complete ? AudioRuntimeAction::Continue
                                             : AudioRuntimeAction::Disable;
}

} // namespace audiodecision
