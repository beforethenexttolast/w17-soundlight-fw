#include <Arduino.h>

#include <atomic>

#include "BuildConfig.hpp"
#include "audio_hal_esp32/Esp32I2sAudio.hpp"
#include "audiodecision/AudioDecision.hpp"
#include "audiostartup/AudioStartup.hpp"
#include "config/PinMap.hpp"
#include "enginesim/EngineSim.hpp"
#include "enginesim/ShowScript.hpp"
#include "lights/LightRenderer.hpp"
#include "lights_hal_esp32/Esp32NeoPixelStrip.hpp"
#include "link2/Link2Codec.hpp"
#include "link2monitor/Link2Monitor.hpp"
#include "soundsynth/EngineSynth.hpp"
#include "soundsynth/SynthProfiles.hpp"

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
// Engine voice: the BOOT profile (the V10 -- see SynthProfiles.hpp), heard
// until the first link2 frame arrives. At runtime the v2 soundProfile byte
// selects the voice (owner decision 15, 2026-08-16): normalized in the
// control tick below, carried in the packed word's profile bits, applied by
// EngineSynth::applyPackedParams on the audio task.
constexpr soundsynth::EngineSynthConfig kSynthConfig = soundsynth::profiles::kDefault;
static_assert(kSynthConfig.valid(), "synth config: partial sum exceeds headroom");
constexpr lights::LightConfig kLightConfig{};
static_assert(kLightConfig.valid(), "light config: power budget or thresholds");

#ifdef W17_SIM_LINK2_FEEDER
// ---- Sim-image self-identification (sl:safety-5) ----------------------------
// A bench image and the delivery image used to be indistinguishable on the
// finished car: same strip, same sounds, and the sim feeder's scripted frames
// say "armed". So the sim build lights one pixel MAGENTA, permanently.
// Magenta is absent from the whole palette (LightRenderer.cpp) -- nothing else
// on this strip ever shows red and blue together without green -- so one glance
// answers "is this the delivery firmware?".
//
// It sits on the rain light's last pixel: dark except during ERS harvest, so
// the marker costs the demo almost nothing. Written AFTER the compositor, so
// it survives every layer including the failsafe hazard: the marker must not
// be something a fault can hide. Pre-rendered through the same cap+gamma the
// renderer applies, so it stays inside the power budget's model.
constexpr uint8_t kSimMarkerPixel =
    static_cast<uint8_t>(kLightConfig.rainLight.start + kLightConfig.rainLight.len - 1);
static_assert(kLightConfig.rainLight.len > 0, "the sim marker needs a rain-light pixel");
static_assert(kSimMarkerPixel < lights::kNumPixels, "sim marker pixel is off the strip");
constexpr uint8_t kSimMarkerFull = lights::renderedDuty(255, kLightConfig.maxBrightness);
constexpr lights::Rgb kSimMarkerColor{kSimMarkerFull, 0, kSimMarkerFull};
#endif

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

#ifndef W17_SIM_LINK2_FEEDER
    // link2 in from board #1 on UART2 (RX only; TX reserved for future ack).
    Serial2.begin(115200, SERIAL_8N1, pinmap::kLink2UartRxPin, /*txPin=*/-1);
#else
    // MUTUAL EXCLUSION (sl:safety-5): the sim image never opens the real link2
    // UART, so a scripted frame and a real one can never interleave into one
    // nonsense state. The RX pin is left unconfigured -- the strongest form of
    // "this build does not listen to board #1". See the marker above for the
    // other half: telling the two images apart on the finished car.
    W17_UART0_PRINTF(
        "[sim] SIMULATION IMAGE (esp32dev_sim): scripted link2 frames, real UART2 NOT read.\n"
        "[sim] One magenta pixel marks this build. Re-flash esp32dev before the car ships.\n");
#endif

    // Audio output startup (SLR-3): drive the I2S bring-up and audio-task
    // creation through the tested sequencing. On any failure the driver is torn
    // down best-effort, no audio task exists, audio stays disabled for this
    // boot, and we emit one diagnostic -- then setup() continues so link2,
    // EngineSim, and the lights still come up normally.
    AudioStartupOps audioOps;
    const audiostartup::AudioStartupResult audioResult = audiostartup::startAudio(audioOps);
    reportAudioStartup(audioResult, audioOps);

    strip.begin();

    // ---- Core-1 loop watchdog (sl:safety-2, owner ruling OD-12 Q2(a)) ----
    //
    // WHAT IT FIXES. The audio side already has a dead-man: params not
    // refreshed for kAudioDeadmanMs and the synth ramps to silence. The LIGHT
    // side has none -- a wedged loop() simply stops calling strip.show(), and
    // WS2812s hold their last frame forever. So before this, a hung control
    // loop went quiet and then sat there showing a perfectly healthy-looking
    // armed teal (or a frozen solid amber), which is the worst possible
    // failure: silent, still, and reassuring. Board #1 has carried a loop
    // TWDT since remediation R5-a; this board was the asymmetric one.
    //
    // WHAT THIS CALL DOES. enableLoopWDT() (esp32-hal.h:111,
    // esp32-hal-misc.c:91-99 of the pinned core) subscribes the Arduino
    // loopTask -- the task this function runs on -- to the ONE global TWDT the
    // framework already starts at boot. The core resets it once per loop pass
    // for us (cores/esp32/main.cpp:47-49), so there is no feed call to forget
    // here and no second watchdog.
    //
    // THE DEADLINE IS THE FRAMEWORK'S, NOT BOARD #1's. sdkconfig ships
    // CONFIG_ESP_TASK_WDT_TIMEOUT_S=5 with CONFIG_ESP_TASK_WDT_PANIC=y, so a
    // wedged loop panics and reboots after ~5 s, against board #1's 2 s.
    // Matching 2 s would mean esp_task_wdt_init(2, true), which UPDATES the
    // single global instance and so also moves the core-0 idle task to a 2 s
    // deadline -- on this board core 0 is the audio pump, and whether it ever
    // starves IDLE0 for 2 s is unmeasurable until the bench (A2 NOT EXECUTED).
    // Taking the framework default adds protection without adding an
    // unvalidated reset risk. [bench-TBD]: an owner call once Phase B can
    // measure it.
    //
    // SYMPTOM LEGEND (so a self-reset is not mistaken for a radio failsafe):
    //   lights blink OFF entirely and the engine re-cranks from silence
    //     = board #2 rebooted itself (this watchdog). The strip is cleared on
    //       boot (Esp32NeoPixelStrip.cpp:8-12) and the monitor re-enters
    //       NeverConnected -> Up, so recovery is clean and obvious.
    //   lights go AMBER HAZARD and the engine falls silent, strip still alive
    //     = link2 staleness / failsafe. Board #2 is fine; board #1 or the
    //       harness is not.
    // Audio leads either way: the 500 ms dead-man mutes long before the ~5 s
    // watchdog fires, so "went quiet, then everything blinked off" is one
    // event, not two.
    enableLoopWDT();
}

void loop() {
    const uint32_t nowMs = millis();

#ifndef W17_SIM_LINK2_FEEDER
    // ---- Drain the link2 UART every pass (never let bytes back up). ----
    while (Serial2.available() > 0) {
        monitor.feedByte(static_cast<uint8_t>(Serial2.read()), nowMs);
    }
#endif

#ifdef W17_SIM_LINK2_FEEDER
    // Standalone bench demo: inject scripted frames through the same monitor
    // path the UART uses. This is the ONLY frame source in this build
    // (sl:safety-5) -- the real UART is not opened, let alone drained.
    {
        static uint8_t simFrame[link2::kFrameLen];
        const size_t n = simfeeder::tick(nowMs, simFrame);
        for (size_t i = 0; i < n; ++i) {
            monitor.feedByte(simFrame[i], nowMs);
        }
    }
#endif

    // ---- 50Hz control tick: monitor -> show script -> engine -> publish. ----
    if (nowMs - lastControlMs >= kControlPeriodMs) {
        lastControlMs = nowMs;
        monitor.poll(nowMs);
        // Showcase (link2 modeFlags bit0): under showcase sound authority the
        // curated idle script substitutes the throttle the wire truthfully
        // carries as 0 (gentle seeded blips, limiter/overrun unreachable by
        // construction -- lib/enginesim/ShowScript.hpp). In every other state
        // -- driving, disarmed, failsafe, low battery, stale link --
        // applyShowScript is a pure pass-through, so the drive path cannot
        // be touched by it. Ignition itself keys on armed || showcase inside
        // EngineSim (D5/D4 gating included).
        engine.update(nowMs, enginesim::applyShowScript(monitor.state(), nowMs));
        const enginesim::EngineState& e = engine.engine();
        // link2 v2 operator sound config rides the same single packed word:
        // the operator volume composes into the state volume here on core 1
        // (failsafe still wins -- Ignition::Off makes the state volume 0 and
        // 0 scales to 0), and the normalized voice profile takes the two
        // profile bits. The cross-core surface stays word + heartbeat.
        const link2::VehicleState& v = monitor.state();
        gSynthParams.store(
            soundsynth::packParams(
                e.engineRpm,
                audiodecision::applyOperatorVolume(
                    audiodecision::synthVolumeFor(e.ignition, e.throttlePercent), v.volume),
                e.ersWhine, e.limiterActive, e.overrunActive,
                audiodecision::normalizeSoundProfile(v.soundProfile)),
            std::memory_order_relaxed);
        gControlHeartbeatMs.store(nowMs, std::memory_order_relaxed);
    }

    // ---- ~30Hz lights ----
    if (nowMs - lastLightsMs >= kLightsPeriodMs) {
        lastLightsMs = nowMs;
        lights::Rgb px[lights::kNumPixels];
        // Ignition comes from the same core-1 EngineSim the control tick
        // updates (the lights animation must key off the real ignition state
        // machine, never re-derive it from `armed`).
        lightRenderer.render(monitor.state(), monitor.status(), engine.engine().ignition, nowMs,
                             px);
#ifdef W17_SIM_LINK2_FEEDER
        px[kSimMarkerPixel] = kSimMarkerColor; // after every layer, hazard included
#endif
        for (uint8_t i = 0; i < lights::kNumPixels; ++i) {
            strip.setPixel(i, px[i].r, px[i].g, px[i].b);
        }
        strip.show();
    }
}
