#pragma once

#include <cstdint>

#include <Adafruit_NeoPixel.h>

namespace lights_hal_esp32 {

// WS2812B strip via Adafruit NeoPixel (which uses the RMT peripheral on
// ESP32 core 2.0.x). Thin wrapper so the pure lights::LightRenderer never
// touches Arduino. Data pin goes through a 330R series resistor at the strip
// with a 1000uF reservoir + 1N5819 on strip VDD (build sheet fixes).
class Esp32NeoPixelStrip {
public:
    Esp32NeoPixelStrip(uint8_t pin, uint16_t numPixels);

    void begin();
    void setPixel(uint16_t i, uint8_t r, uint8_t g, uint8_t b);
    void show();

private:
    Adafruit_NeoPixel strip_;
};

} // namespace lights_hal_esp32
