#include "lights_hal_esp32/Esp32NeoPixelStrip.hpp"

namespace lights_hal_esp32 {

Esp32NeoPixelStrip::Esp32NeoPixelStrip(uint8_t pin, uint16_t numPixels)
    : strip_(numPixels, pin, NEO_GRB + NEO_KHZ800) {}

void Esp32NeoPixelStrip::begin() {
    strip_.begin();
    strip_.clear();
    strip_.show();
}

void Esp32NeoPixelStrip::setPixel(uint16_t i, uint8_t r, uint8_t g, uint8_t b) {
    strip_.setPixelColor(i, strip_.Color(r, g, b));
}

void Esp32NeoPixelStrip::show() { strip_.show(); }

} // namespace lights_hal_esp32
