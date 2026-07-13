#pragma once

#include <cstddef>
#include <cstdint>

namespace audio_hal_esp32 {

// I2S output to the MAX98357A via the IDF 4.4 legacy driver (driver/i2s.h),
// which is what Arduino-ESP32 2.0.x ships. 16-bit, stereo frames at the
// synth's sample rate. We render mono and duplicate to L+R so the output is
// identical under every MAX98357A SD_MODE channel selection.
//
// Startup is split into the individual driver operations so main.cpp can drive
// them through the tested audiostartup sequencing and disable audio (rather
// than run wedged) if any step fails (SLR-3). Each returns true only on the
// IDF's documented success value (ESP_OK); the failing esp_err_t is captured in
// lastError() for a one-shot startup diagnostic. write() blocks until the
// samples are queued into the DMA ring (self-pacing the audio pump task).
class Esp32I2sAudio {
public:
    Esp32I2sAudio(uint8_t bclkPin, uint8_t lrclkPin, uint8_t dataPin, uint32_t sampleRateHz);

    // Startup steps, in order. Each returns true iff the underlying IDF call
    // returned ESP_OK; on failure lastError() holds the esp_err_t.
    bool installDriver();
    bool configurePins();
    bool clearDma();

    // Best-effort teardown after a post-install startup failure. Idempotently
    // safe from the caller's view: audiostartup calls it at most once and never
    // after a successful start. The uninstall result is intentionally ignored
    // -- see the .cpp for why it is non-actionable in degraded mode.
    void uninstallDriver();

    // esp_err_t (as int32_t) from the most recent failed startup step; ESP_OK
    // (0) if none has failed. Kept ESP-IDF-header-free so the type never leaks.
    int32_t lastError() const { return lastError_; }

    // Blocking write of `frameCount` stereo frames (2 int16 each). Returns
    // frames actually written.
    size_t write(const int16_t* stereoFrames, size_t frameCount);

private:
    uint8_t bclkPin_;
    uint8_t lrclkPin_;
    uint8_t dataPin_;
    uint32_t sampleRateHz_;
    int32_t lastError_ = 0; // ESP_OK
};

} // namespace audio_hal_esp32
