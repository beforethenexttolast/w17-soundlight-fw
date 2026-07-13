#pragma once

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

} // namespace audiodecision
