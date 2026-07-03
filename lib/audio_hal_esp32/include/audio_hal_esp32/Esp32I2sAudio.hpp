#pragma once

#include <cstddef>
#include <cstdint>

namespace audio_hal_esp32 {

// I2S output to the MAX98357A via the IDF 4.4 legacy driver (driver/i2s.h),
// which is what Arduino-ESP32 2.0.x ships. 16-bit, stereo frames at the
// synth's sample rate. We render mono and duplicate to L+R so the output is
// identical under every MAX98357A SD_MODE channel selection.
//
// begin() installs the driver + routes the pins; write() blocks until the
// samples are queued into the DMA ring (self-pacing the audio pump task).
class Esp32I2sAudio {
public:
    Esp32I2sAudio(uint8_t bclkPin, uint8_t lrclkPin, uint8_t dataPin, uint32_t sampleRateHz);

    void begin();

    // Blocking write of `frameCount` stereo frames (2 int16 each). Returns
    // frames actually written.
    size_t write(const int16_t* stereoFrames, size_t frameCount);

private:
    uint8_t bclkPin_;
    uint8_t lrclkPin_;
    uint8_t dataPin_;
    uint32_t sampleRateHz_;
};

} // namespace audio_hal_esp32
