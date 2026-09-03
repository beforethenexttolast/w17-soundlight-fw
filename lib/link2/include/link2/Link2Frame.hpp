#pragma once

#include <cstddef>
#include <cstdint>

// link2 wire protocol v2: control board (ESP32 #1) -> sound/light board
// (ESP32 #2), one-way UART, 115200 8N1, nominal 20 Hz.
//
// Frame: [0]=start 0xA5 | [1]=length (payload bytes) | [2..2+len)=payload | [last]=crc8
//   - crc8: poly 0xD5, computed over [length + payload] (start byte excluded).
//   - Receivers MUST hard-reject a length byte they don't support as soon as
//     it arrives, and MUST treat "no CRC-valid frame for 500 ms" as link loss
//     and fall back to their OWN documented safe state -- on a one-way link a
//     cut wire is otherwise indistinguishable from "last state persists".
//     The protocol does not dictate that state; board #2 (w17-soundlight-fw)
//     implements it as a per-field projection that zeroes the commands and
//     asserts failsafe, which leaves the engine SILENT (not idling: its
//     ignition authority is armed || showcase, and both are cleared) under an
//     all-amber hazard blink.
//
// Payload v2 (all multi-byte fields little-endian):
//   [0] version = 2
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
//   [10] driveMode       0 = TRAINING, 1 = RACE (gearbox), 2 = ERS (gearbox+ERS deploy)
//   [11] soundProfile    engine voice: 0 = V10 (default), 1 = V6 turbo-hybrid;
//                        values >= kSoundProfileCount reserved -- receivers
//                        fall back to 0 (V10), never reject the frame
//   [12] volume          0..100 engine-sound level; 0 = TRUE silence, receivers
//                        clamp >100 to 100; scales sound only, never lights
//   [13] modeFlags       bit0 showcase -- LIVE: board #1 booted in its
//                        stationary-demo SHOWCASE state (set in every frame
//                        of such a boot, never in a DRIVE boot; every other
//                        field stays truthful). Receivers key ignition/
//                        presentation on it and treat it as command-class on
//                        staleness (a stale showcase must not outlive the
//                        link). bit1 awaitingController -- reserved for the
//                        BT show-off pairing surface, always 0 today; bits
//                        2-7 spare (sender writes 0, receivers mask/ignore,
//                        never reject)
//
// v1 -> v2 (2026-08-17): appended soundProfile + volume (vision decision 15)
// and modeFlags (owner 2026-08-17: showcase + BT show-off both wanted flags
// bit7, resolved with a dedicated byte). Same framing/CRC; length byte
// 11 -> 14, version byte 1 -> 2. Because receivers hard-reject unsupported
// length bytes, a version-mismatched pair of boards is safe but
// non-functional (permanent staleness failsafe), so BOTH boards must be
// flashed together -- there is no mixed-version interop.
//
// Full spec with a worked example: docs/link2_protocol.md.

namespace link2 {

inline constexpr uint8_t kStartByte = 0xA5;
inline constexpr uint8_t kProtocolVersion = 2;
inline constexpr size_t kPayloadLen = 14;
inline constexpr size_t kFrameLen = 3 + kPayloadLen; // start + length + payload + crc

// Flag bit positions (payload byte [3]).
inline constexpr uint8_t kFlagBraking = 1u << 0;
inline constexpr uint8_t kFlagReverse = 1u << 1; // reserved since v1, always 0
inline constexpr uint8_t kFlagDrsOpen = 1u << 2;
inline constexpr uint8_t kFlagArmed = 1u << 3;
inline constexpr uint8_t kFlagFailsafe = 1u << 4;
inline constexpr uint8_t kFlagLowBattery = 1u << 5;
inline constexpr uint8_t kFlagErsDeploying = 1u << 6;

// Sound profile wire values (payload byte [11], v2). kSoundProfileCount is
// the receiver fallback boundary: any value >= it selects V10 -- a voice
// fallback, NEVER a frame rejection -- so a receiver that predates a future
// voice keeps making sound instead of going mute (mirrors driveMode's
// treat-unknown-as-RACE rule).
inline constexpr uint8_t kSoundProfileV10 = 0;      // default engine voice
inline constexpr uint8_t kSoundProfileV6Hybrid = 1; // V6 turbo-hybrid voice
inline constexpr uint8_t kSoundProfileCount = 2;    // first reserved value

// Volume (payload byte [12], v2): 0..100 engine-sound level, 0 = TRUE
// silence, receivers clamp >100 to 100. Default 80 -- loud enough to be the
// showpiece out of the box, deliberately below full scale so "louder" stays
// available on request instead of shipping the synth pinned at its peak.
inline constexpr uint8_t kVolumeMax = 100;
inline constexpr uint8_t kDefaultVolume = 80;

// modeFlags bit positions (payload byte [13], v2). The byte was introduced
// EMPTY in the v2 bump so that switching each accepted mode on later is a
// sender-behavior change on an already-flashed wire format, not another
// coordinated protocol bump. (Owner decision 2026-08-17: showcase D2 and
// BT-7 both wanted flags bit7 -- collision resolved with this dedicated
// byte; flags bit7 stays reserved.)
//   bit0 showcase: LIVE since the showcase-mode wave. Truth on the wire --
//     the bit says exactly "board #1 booted in SHOWCASE" (boot-selected,
//     constant all session); armed/throttle/battery/steering stay truthful
//     and the failsafe flag follows the showcase-scoped D4 rule in
//     docs/link2_protocol.md. Receivers treat it as command-class: zeroed
//     by the staleness projection, like `armed`.
//   bit1 awaitingController: still reserved for the BT show-off pairing
//     surface (§6.3), always 0 until that mode ships.
// Bits 2-7 are spare: sender writes 0, receivers mask/ignore, never reject.
inline constexpr uint8_t kModeFlagShowcase = 1u << 0;           // board-1 SHOWCASE boot state
inline constexpr uint8_t kModeFlagAwaitingController = 1u << 1; // future BT pairing surface (§6.3)

struct VehicleState {
    int8_t throttlePercent = 0;
    int8_t steeringPercent = 0;
    bool braking = false;
    bool reverse = false; // reserved since v1, always false
    bool drsOpen = false;
    bool armed = false;
    bool failsafe = true; // boot-safe default: never report a phantom Active
    bool lowBattery = false;
    bool ersDeploying = false;
    uint8_t gear = 1; // 1-based display gear
    uint16_t rpm = 0;
    uint16_t batteryMv = 0;
    uint8_t ersPercent = 100; // store starts full
    uint8_t driveMode = 1;    // 0 TRAINING / 1 RACE (gearbox) / 2 ERS (gearbox+ERS)
    uint8_t soundProfile = kSoundProfileV10; // v2: engine voice; >= kSoundProfileCount reserved
    uint8_t volume = kDefaultVolume;         // v2: 0..100 sound level, 0 = true silence
    bool showcase = false;           // v2 modeFlags bit0 -- board-1 SHOWCASE boot state
    bool awaitingController = false; // v2 modeFlags bit1 -- reserved, always false today
};

enum class DecodeResult : uint8_t {
    Ok,
    BadStart,    // data[0] != kStartByte
    BadLength,   // length byte/buffer size unsupported
    CrcMismatch, // checked BEFORE version, so BadVersion means a well-formed
    BadVersion,  // frame from a newer sender, not corruption
};

} // namespace link2
