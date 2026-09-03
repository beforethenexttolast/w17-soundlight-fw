#pragma once

#include <cstddef>
#include <cstdint>

#include "enginesim/EngineSim.hpp"
#include "link2/Link2Frame.hpp" // v2 soundProfile/volume wire constants

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
//
// PRECONDITION: throttlePercent <= 100. This function is NOT total over its
// argument type and deliberately does not clamp -- the receiver-side bound
// lives once at EngineSim::clampThrottle, where the wire value is adopted
// (sl:correctness-3). 101 here yields 90 + 101*165/100 = 256, which narrows
// to 0: bit-exact silence at what the frame calls full throttle. Pinned in
// test_audiodecision so the precondition cannot rot into a surprise.
constexpr uint8_t synthVolumeFor(enginesim::Ignition ignition, uint8_t throttlePercent) {
    if (ignition == enginesim::Ignition::Off) return 0;
    if (ignition == enginesim::Ignition::Cranking) return 70;
    return static_cast<uint8_t>(90 + throttlePercent * 165 / 100); // 90..255
}

// ---- link2 v2 operator sound config (vision decision 15) -------------------

// Wire soundProfile -> the value packed into the synth-param word's two
// profile bits. THE normative fallback gate: every reserved value
// (>= link2::kSoundProfileCount) becomes V10 here, on the control core,
// BEFORE the 2-bit pack -- masking a reserved value like 5 (0b101) into two
// bits without this rule would alias it onto the V6, violating the
// protocol's fall-back-to-V10 obligation. (soundsynth::profiles::
// voiceForProfile is total too, as defense in depth.)
constexpr uint8_t normalizeSoundProfile(uint8_t wireProfile) {
    return wireProfile < link2::kSoundProfileCount ? wireProfile : link2::kSoundProfileV10;
}

// Composes the operator volume (link2 v2 `volume` byte, 0..100) into the
// engine-state volume (synthVolumeFor's 0..255), producing the byte packed
// into the synth-param word. The synth applies that byte at its final gain
// stage (`sample = sample * vol / 255`, the last multiplicative stage before
// the int16 clamp), so operator scaling takes effect exactly there, in
// integer math, at 8-bit gain resolution.
//
// The mapping is LINEAR (truncating integer multiply), not stepped/log,
// deliberately:
//   - the two ends carry the product requirements exactly: 0 -> 0 (true
//     silence, bit-exact) and 100 -> identity (bit-transparent, the
//     pre-v2 behavior);
//   - it is monotone at every step in between, which is all a pit-lane
//     console knob set to a handful of values (0 mute / ~25 quiet indoor /
//     80 default / 100 max) needs -- a perceptual (log) taper would buy
//     nothing an operator would notice at those set points and would cost a
//     LUT or pow-approx in an integer-only path;
//   - composing HERE, on the control core, keeps the cross-core surface at
//     the single packed word (CLAUDE.md cross-core rule) -- a separate gain
//     stage inside render() would have needed a second shared channel.
// Values above link2::kVolumeMax clamp to 100 first (receiver obligation;
// the sender never emits them, but the wire could).
//
// Precedence note: failsafe/staleness silencing needs no special case here.
// Link loss drives Ignition to Off upstream, so stateVolume arrives as 0 and
// 0 * anything = 0 -- volume can never resurrect a silenced engine. The
// audio task's dead-man additionally bypasses this path entirely.
constexpr uint8_t applyOperatorVolume(uint8_t stateVolume, uint8_t operatorVolume) {
    const uint8_t op =
        operatorVolume > link2::kVolumeMax ? link2::kVolumeMax : operatorVolume;
    return static_cast<uint8_t>(stateVolume * op / link2::kVolumeMax);
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
