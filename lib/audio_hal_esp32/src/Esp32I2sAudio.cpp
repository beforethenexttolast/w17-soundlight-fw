#include "audio_hal_esp32/Esp32I2sAudio.hpp"

#include <Arduino.h>
#include <driver/i2s.h>

namespace audio_hal_esp32 {

namespace {
constexpr i2s_port_t kPort = I2S_NUM_0;
constexpr int kDmaBufCount = 6;   // ~70ms of buffer at 22050Hz / 256 frames
constexpr int kDmaBufLen = 256;   // frames per DMA buffer
} // namespace

Esp32I2sAudio::Esp32I2sAudio(uint8_t bclkPin, uint8_t lrclkPin, uint8_t dataPin,
                             uint32_t sampleRateHz)
    : bclkPin_(bclkPin), lrclkPin_(lrclkPin), dataPin_(dataPin), sampleRateHz_(sampleRateHz) {}

void Esp32I2sAudio::begin() {
    i2s_config_t cfg = {};
    cfg.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX);
    cfg.sample_rate = sampleRateHz_;
    cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    // Transmit both channels; the synth duplicates mono into L+R.
    cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count = kDmaBufCount;
    cfg.dma_buf_len = kDmaBufLen;
    cfg.use_apll = false;
    cfg.tx_desc_auto_clear = true;

    i2s_pin_config_t pins = {};
    pins.bck_io_num = bclkPin_;
    pins.ws_io_num = lrclkPin_;
    pins.data_out_num = dataPin_;
    pins.data_in_num = I2S_PIN_NO_CHANGE;

    i2s_driver_install(kPort, &cfg, 0, nullptr);
    i2s_set_pin(kPort, &pins);
    i2s_zero_dma_buffer(kPort);
}

size_t Esp32I2sAudio::write(const int16_t* stereoFrames, size_t frameCount) {
    size_t bytesWritten = 0;
    const size_t bytes = frameCount * 2 * sizeof(int16_t);
    i2s_write(kPort, stereoFrames, bytes, &bytesWritten, portMAX_DELAY);
    return bytesWritten / (2 * sizeof(int16_t));
}

} // namespace audio_hal_esp32
