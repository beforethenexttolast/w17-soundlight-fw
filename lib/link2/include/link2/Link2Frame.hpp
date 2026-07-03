#pragma once

#include <cstddef>
#include <cstdint>

// link2 wire protocol v1: control board (ESP32 #1) -> sound/light board
// (ESP32 #2), one-way UART, 115200 8N1, nominal 20 Hz.
//
// Frame: [0]=start 0xA5 | [1]=length (payload bytes) | [2..2+len)=payload | [last]=crc8
//   - crc8: poly 0xD5, computed over [length + payload] (start byte excluded).
//   - Receivers MUST hard-reject a length byte they don't support as soon as
//     it arrives, and MUST treat "no CRC-valid frame for 500 ms" as link loss
//     (engine to idle, hazard blink) -- on a one-way link a cut wire is
//     otherwise indistinguishable from "last state persists".
//
// Payload v1 (all multi-byte fields little-endian):
//   [0] version = 1
//   [1] throttlePercent  int8 -100..100, what the ESC is ACTUALLY commanded
//                        (0 while disarmed/failsafe, incl. ERS boost), so
//                        engine sound tracks the motor, not the stick
//   [2] steeringPercent  int8 -100..100 (for turn indicators)
//   [3] flags            bit0 braking, bit1 reverse (reserved, always 0:
//                        the ESC runs forward/brake), bit2 drsOpen,
//                        bit3 armed, bit4 failsafe, bit5 lowBattery,
//                        bit6 ersDeploying, bit7 reserved (sender writes 0,
//                        receivers mask)
//   [4] gear             1-based display gear
//   [5-6] rpm            uint16, WHEEL rpm (not engine rpm)
//   [7-8] batteryMv      uint16, 2S pack millivolts
//   [9] ersPercent       0..100, ERS energy store
//   [10] driveMode       0 = Training, 1 = Gearbox, 2 = Gearbox+ERS
//
// Full spec with a worked example: docs/link2_protocol.md.

namespace link2 {

inline constexpr uint8_t kStartByte = 0xA5;
inline constexpr uint8_t kProtocolVersion = 1;
inline constexpr size_t kPayloadLen = 11;
inline constexpr size_t kFrameLen = 3 + kPayloadLen; // start + length + payload + crc

// Flag bit positions (payload byte [3]).
inline constexpr uint8_t kFlagBraking = 1u << 0;
inline constexpr uint8_t kFlagReverse = 1u << 1; // reserved, always 0 in v1
inline constexpr uint8_t kFlagDrsOpen = 1u << 2;
inline constexpr uint8_t kFlagArmed = 1u << 3;
inline constexpr uint8_t kFlagFailsafe = 1u << 4;
inline constexpr uint8_t kFlagLowBattery = 1u << 5;
inline constexpr uint8_t kFlagErsDeploying = 1u << 6;

struct VehicleState {
    int8_t throttlePercent = 0;
    int8_t steeringPercent = 0;
    bool braking = false;
    bool reverse = false; // reserved, always false in v1
    bool drsOpen = false;
    bool armed = false;
    bool failsafe = true; // boot-safe default: never report a phantom Active
    bool lowBattery = false;
    bool ersDeploying = false;
    uint8_t gear = 1; // 1-based display gear
    uint16_t rpm = 0;
    uint16_t batteryMv = 0;
    uint8_t ersPercent = 100; // store starts full
    uint8_t driveMode = 1;    // 0 Training / 1 Gearbox / 2 Gearbox+ERS
};

enum class DecodeResult : uint8_t {
    Ok,
    BadStart,    // data[0] != kStartByte
    BadLength,   // length byte/buffer size unsupported
    CrcMismatch, // checked BEFORE version, so BadVersion means a well-formed
    BadVersion,  // frame from a newer sender, not corruption
};

} // namespace link2
