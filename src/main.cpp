#include <Arduino.h>

#include <atomic>

#include "BuildConfig.hpp"
#include "audio_hal_esp32/Esp32I2sAudio.hpp"
#include "audiodecision/AudioDecision.hpp"
#include "audiostartup/AudioStartup.hpp"
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

// ---- Audio pump: core 0, blocks in i2s.write, self-paced ----
void audioTask(void*) {
    constexpr size_t kFrames = 256;
    static int16_t buf[kFrames * 2];
    for (;;) {
        // Dead-man: if the control loop hasn't ticked recently, force silent
        // params regardless of the last packed value.
        const uint32_t now = millis();
        const uint32_t hb = gControlHeartbeatMs.load(std::memory_order_relaxed);
        if (audiodecision::isAudioHeartbeatStale(now, hb, kAudioDeadmanMs)) {
            synth.setParams(0, 0, false, false, false);
        } else {
            synth.applyPackedParams(gSynthParams.load(std::memory_order_relaxed));
        }
        synth.render(buf, kFrames);

        // Runtime I2S write handling (SLR-4). A completed write that reports a
        // non-success status, or success with a byte count other than the 1024
        // requested, permanently disables audio for this boot: emit exactly one
        // diagnostic and delete this task. No retry, no driver uninstall, no
        // reset -- the task's non-existence IS the audio-disabled state, and
        // link2/EngineSim/lights/failsafe on core 1 keep running. This does NOT
        // detect an i2s_write that blocks forever under portMAX_DELAY.
        const audio_hal_esp32::WriteResult wr = i2s.write(buf, kFrames);
        if (audiodecision::runtimeActionFor(
                audiodecision::classifyWrite(wr.status, wr.requestedBytes, wr.bytesWritten)) ==
            audiodecision::AudioRuntimeAction::Disable) {
            W17_UART0_PRINTF("audio disabled: runtime write fault (err 0x%x, requested %u, wrote %u)\n",
                             static_cast<unsigned>(wr.status),
                             static_cast<unsigned>(wr.requestedBytes),
                             static_cast<unsigned>(wr.bytesWritten));
            vTaskDelete(nullptr); // never returns; no code runs after this
        }
    }
}

// ---- Audio-device startup adapter (SLR-3) ----
// Binds the pure audiostartup sequencing to the real IDF / FreeRTOS calls.
// Each I2S op forwards to the HAL (true only on ESP_OK); createTask maps
// pdPASS -> true and keeps the audio task parameters byte-for-byte identical to
// the historical unchecked call. taskResult retains the BaseType_t for the
// one-shot failure diagnostic. No allocation, no dynamic strings.
struct AudioStartupOps {
    BaseType_t taskResult = pdPASS;

    bool installDriver() { return i2s.installDriver(); }
    bool configurePins() { return i2s.configurePins(); }
    bool clearDma() { return i2s.clearDma(); }
    bool createTask() {
        // Audio pump on core 0 (Arduino loop owns core 1; no WiFi/BT here).
        taskResult =
            xTaskCreatePinnedToCore(audioTask, "audio", 4096, nullptr, 5, nullptr, 0);
        return taskResult == pdPASS;
    }
    void uninstallDriver() { i2s.uninstallDriver(); }
};

// Emit exactly one concise startup diagnostic for a failed audio bring-up
// (none on success -- the firmware prints nothing on a healthy boot). Includes
// the underlying esp_err_t / task return code. No dynamic String.
void reportAudioStartup(audiostartup::AudioStartupResult result,
                        [[maybe_unused]] const AudioStartupOps& ops) {
    switch (result) {
        case audiostartup::AudioStartupResult::Ready:
            return;
        case audiostartup::AudioStartupResult::DriverInstallFailed:
            W17_UART0_PRINTF("audio disabled: i2s driver install failed (err 0x%x)\n",
                             static_cast<unsigned>(i2s.lastError()));
            return;
        case audiostartup::AudioStartupResult::PinConfigurationFailed:
            W17_UART0_PRINTF("audio disabled: i2s pin setup failed (err 0x%x)\n",
                             static_cast<unsigned>(i2s.lastError()));
            return;
        case audiostartup::AudioStartupResult::DmaClearFailed:
            W17_UART0_PRINTF("audio disabled: i2s DMA clear failed (err 0x%x)\n",
                             static_cast<unsigned>(i2s.lastError()));
            return;
        case audiostartup::AudioStartupResult::TaskCreationFailed:
            W17_UART0_PRINTF("audio disabled: task creation failed (rc %d)\n",
                             static_cast<int>(ops.taskResult));
            return;
    }
}

uint32_t lastControlMs = 0;
uint32_t lastLightsMs = 0;

} // namespace

void setup() {
#if W17_UART0_DIAGNOSTICS
    // Application UART0 for diagnostics only; compiled out of the delivery
    // build (finding SL-3). ROM/bootloader output on UART0 is outside this gate.
    Serial.begin(115200);
#endif

    // link2 in from board #1 on UART2 (RX only; TX reserved for future ack).
    Serial2.begin(115200, SERIAL_8N1, pinmap::kLink2UartRxPin, /*txPin=*/-1);

    // Audio output startup (SLR-3): drive the I2S bring-up and audio-task
    // creation through the tested sequencing. On any failure the driver is torn
    // down best-effort, no audio task exists, audio stays disabled for this
    // boot, and we emit one diagnostic -- then setup() continues so link2,
    // EngineSim, and the lights still come up normally.
    AudioStartupOps audioOps;
    const audiostartup::AudioStartupResult audioResult = audiostartup::startAudio(audioOps);
    reportAudioStartup(audioResult, audioOps);

    strip.begin();
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
            soundsynth::packParams(e.engineRpm,
                                   audiodecision::synthVolumeFor(e.ignition, e.throttlePercent),
                                   e.ersWhine, e.limiterActive, e.overrunActive),
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
