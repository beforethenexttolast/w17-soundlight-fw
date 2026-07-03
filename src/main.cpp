#include <Arduino.h>

#include <atomic>

#include "audio_hal_esp32/Esp32I2sAudio.hpp"
#include "config/PinMap.hpp"
#include "enginesim/EngineSim.hpp"
#include "lights/LightRenderer.hpp"
#include "lights_hal_esp32/Esp32NeoPixelStrip.hpp"
#include "link2/Link2Codec.hpp"
#include "link2monitor/Link2Monitor.hpp"
#include "soundsynth/EngineSynth.hpp"

#ifdef W17_SIM_LINK2_FEEDER
#include "SimLink2Feeder.hpp"
#endif

namespace {

// ---- Cross-core surface (the ONLY shared state; see CLAUDE.md) ----
// Packed synth params, written by the control loop (core 1), read by the
// audio pump (core 0). One atomic word => torn-free, lock-free.
std::atomic<uint32_t> gSynthParams{0};
// Heartbeat: control loop stamps millis() each tick; the audio task ramps to
// silence if it goes stale (a wedged control loop must not scream forever).
std::atomic<uint32_t> gControlHeartbeatMs{0};

// ---- Config (validated at compile time) ----
constexpr link2monitor::Link2MonitorConfig kMonitorConfig{};
static_assert(kMonitorConfig.valid(), "monitor config");
constexpr enginesim::EngineSimConfig kEngineConfig{};
static_assert(kEngineConfig.valid(), "engine config");
constexpr soundsynth::EngineSynthConfig kSynthConfig{};
static_assert(kSynthConfig.valid(), "synth config: partial sum exceeds headroom");
constexpr lights::LightConfig kLightConfig{};
static_assert(kLightConfig.valid(), "light config: power budget or thresholds");

// ---- Core-1 (control) objects ----
link2monitor::Link2Monitor monitor(kMonitorConfig);
enginesim::EngineSim engine(kEngineConfig);
lights::LightRenderer lightRenderer(kLightConfig);

// ---- Core-0 (audio) objects ----
soundsynth::EngineSynth synth(kSynthConfig);
audio_hal_esp32::Esp32I2sAudio i2s(pinmap::kI2sBclkPin, pinmap::kI2sLrclkPin,
                                    pinmap::kI2sDataPin, soundsynth::kSampleRateHz);

lights_hal_esp32::Esp32NeoPixelStrip strip(pinmap::kLedStripPin, lights::kNumPixels);

constexpr uint32_t kControlPeriodMs = 20; // 50Hz
constexpr uint32_t kLightsPeriodMs = 33;  // ~30Hz
constexpr uint32_t kAudioDeadmanMs = 500; // control-loop staleness -> mute

// Maps the engine state to a synth volume: silent Off, quiet crank, rising
// with throttle while Running.
uint8_t volumeFor(const enginesim::EngineState& e) {
    if (e.ignition == enginesim::Ignition::Off) return 0;
    if (e.ignition == enginesim::Ignition::Cranking) return 70;
    return static_cast<uint8_t>(90 + e.throttlePercent * 165 / 100); // 90..255
}

// ---- Audio pump: core 0, blocks in i2s.write, self-paced ----
void audioTask(void*) {
    constexpr size_t kFrames = 256;
    static int16_t buf[kFrames * 2];
    for (;;) {
        // Dead-man: if the control loop hasn't ticked recently, force silent
        // params regardless of the last packed value.
        const uint32_t now = millis();
        const uint32_t hb = gControlHeartbeatMs.load(std::memory_order_relaxed);
        if (now - hb > kAudioDeadmanMs) {
            synth.setParams(0, 0, false, false, false);
        } else {
            synth.applyPackedParams(gSynthParams.load(std::memory_order_relaxed));
        }
        synth.render(buf, kFrames);
        i2s.write(buf, kFrames);
    }
}

uint32_t lastControlMs = 0;
uint32_t lastLightsMs = 0;

} // namespace

void setup() {
    Serial.begin(115200);

    // link2 in from board #1 on UART2 (RX only; TX reserved for future ack).
    Serial2.begin(115200, SERIAL_8N1, pinmap::kLink2UartRxPin, /*txPin=*/-1);

    i2s.begin();
    strip.begin();

    // Audio pump on core 0 (Arduino loop owns core 1; no WiFi/BT here).
    xTaskCreatePinnedToCore(audioTask, "audio", 4096, nullptr, 5, nullptr, 0);
}

void loop() {
    const uint32_t nowMs = millis();

    // ---- Drain the link2 UART every pass (never let bytes back up). ----
    while (Serial2.available() > 0) {
        monitor.feedByte(static_cast<uint8_t>(Serial2.read()), nowMs);
    }

#ifdef W17_SIM_LINK2_FEEDER
    // Standalone bench demo: inject scripted frames through the same monitor
    // path the UART uses.
    {
        static uint8_t simFrame[link2::kFrameLen];
        const size_t n = simfeeder::tick(nowMs, simFrame);
        for (size_t i = 0; i < n; ++i) {
            monitor.feedByte(simFrame[i], nowMs);
        }
    }
#endif

    // ---- 50Hz control tick: monitor -> engine -> publish synth params. ----
    if (nowMs - lastControlMs >= kControlPeriodMs) {
        lastControlMs = nowMs;
        monitor.poll(nowMs);
        engine.update(nowMs, monitor.state());
        const enginesim::EngineState& e = engine.engine();
        gSynthParams.store(
            soundsynth::packParams(e.engineRpm, volumeFor(e), e.ersWhine, e.limiterActive,
                                   e.overrunActive),
            std::memory_order_relaxed);
        gControlHeartbeatMs.store(nowMs, std::memory_order_relaxed);
    }

    // ---- ~30Hz lights ----
    if (nowMs - lastLightsMs >= kLightsPeriodMs) {
        lastLightsMs = nowMs;
        lights::Rgb px[lights::kNumPixels];
        lightRenderer.render(monitor.state(), monitor.status(), nowMs, px);
        for (uint8_t i = 0; i < lights::kNumPixels; ++i) {
            strip.setPixel(i, px[i].r, px[i].g, px[i].b);
        }
        strip.show();
    }
}
