#pragma once

#include <cstdint>

// All GPIO assignments for ESP32 #2 "sound + light". This board owns its own
// pin choices (nothing here is dictated by board #1). All non-strapping,
// none input-only. Bench-verify before soldering.

namespace pinmap {

// UART from board #1 (one-way link2 stream, 115200 8N1, common ground).
inline constexpr uint8_t kLink2UartRxPin = 16;
// Reserved for the protocol's future ack channel -- NOT opened in firmware.
inline constexpr uint8_t kLink2UartTxPinReserved = 17;

// I2S to the MAX98357A -- the canonical Adafruit hookup.
inline constexpr uint8_t kI2sBclkPin = 26;
inline constexpr uint8_t kI2sLrclkPin = 25;
inline constexpr uint8_t kI2sDataPin = 22;
// MAX98357A straps (documented, not driven): GAIN floating = 9dB (start
// there; GND = 12dB, VDD = 6dB). SD_MODE strapped high = (L+R)/2 output --
// we transmit duplicated stereo so every SD_MODE selection sounds identical.
// If the power-on pop annoys on the bench, drive SD_MODE from a spare GPIO
// with a delay after the I2S clocks are stable.

// WS2812B strip data, through the 330R series resistor at the strip end
// (build sheet fixes: 1000uF across strip 5V/GND + 1N5819 on strip VDD).
inline constexpr uint8_t kLedStripPin = 4;

} // namespace pinmap
