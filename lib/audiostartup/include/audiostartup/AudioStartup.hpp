#pragma once

#include <cstdint>

// Pure audio-device startup sequencing + cleanup policy, shared verbatim by
// src/main.cpp and the native tests (SLR-3). This owns the *decision* order --
// install -> pins -> DMA clear -> task, with best-effort uninstall on any
// failure that happens after a successful install -- while the real ESP-IDF /
// FreeRTOS calls live behind an Ops adapter. No hardware, no Arduino / ESP-IDF
// headers, no allocation, no exceptions: the same sequencing the firmware runs
// is exercised natively by a fake Ops.
namespace audiostartup {

// Distinct outcome for every mandatory startup step, plus explicit success.
// A failed startup means audio output is disabled for the current boot; the
// rest of the system (link2, EngineSim, lights, the Arduino loop) continues.
enum class AudioStartupResult : uint8_t {
    Ready,                  // all four steps succeeded; audio task is running
    DriverInstallFailed,    // i2s_driver_install did not return ESP_OK
    PinConfigurationFailed, // i2s_set_pin did not return ESP_OK
    DmaClearFailed,         // i2s_zero_dma_buffer did not return ESP_OK
    TaskCreationFailed,     // xTaskCreatePinnedToCore did not return pdPASS
};

// Drive audio startup through `ops`, stopping at the first failed step.
//
// Ops must provide (each I2S/task op returns true only on the framework's
// documented success value; cleanup is best-effort and returns nothing):
//     bool installDriver();    // i2s_driver_install(...)  == ESP_OK
//     bool configurePins();    // i2s_set_pin(...)         == ESP_OK
//     bool clearDma();         // i2s_zero_dma_buffer(...) == ESP_OK
//     bool createTask();       // xTaskCreatePinnedToCore(...) == pdPASS
//     void uninstallDriver();  // i2s_driver_uninstall(...), best-effort
//
// Cleanup contract: uninstallDriver() is called exactly once, and only on a
// failure that occurs *after* a successful install (pin / DMA / task). It is
// never called when install itself failed (nothing to tear down), and never
// after a successful task creation (the running task owns the driver). Its
// result is intentionally not consulted here -- see the adapter.
template <typename Ops>
AudioStartupResult startAudio(Ops& ops) {
    if (!ops.installDriver()) {
        // Driver was never installed -- no cleanup, nothing to undo.
        return AudioStartupResult::DriverInstallFailed;
    }
    if (!ops.configurePins()) {
        ops.uninstallDriver();
        return AudioStartupResult::PinConfigurationFailed;
    }
    if (!ops.clearDma()) {
        ops.uninstallDriver();
        return AudioStartupResult::DmaClearFailed;
    }
    if (!ops.createTask()) {
        ops.uninstallDriver();
        return AudioStartupResult::TaskCreationFailed;
    }
    return AudioStartupResult::Ready;
}

} // namespace audiostartup
