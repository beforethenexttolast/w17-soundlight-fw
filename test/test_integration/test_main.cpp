#include <unity.h>

// Pure end-to-end chain, native: encoded link2 bytes -> Link2Monitor ->
// EngineSim -> EngineSynth params + render, and -> LightRenderer. Everything
// here is hardware-free, so this integration path runs on the host and is the
// cheapest confidence available before the bench.

#include "audiodecision/AudioDecision.hpp"
#include "enginesim/EngineSim.hpp"
#include "lights/LightRenderer.hpp"
#include "link2/Link2Codec.hpp"
#include "link2monitor/Link2Monitor.hpp"
#include "soundsynth/EngineSynth.hpp"

using enginesim::EngineSim;
using enginesim::Ignition;
using lights::kNumPixels;
using lights::LightRenderer;
using lights::Rgb;
using link2::VehicleState;
using link2monitor::Link2Monitor;
using link2monitor::LinkStatus;
using soundsynth::EngineSynth;

namespace {

// Volume comes from the production decision (audiodecision::synthVolumeFor),
// the same function src/main.cpp packs into the synth-param word -- no
// test-local copy of the mapping or its constants.
uint8_t volumeFor(const enginesim::EngineState& e) {
    return audiodecision::synthVolumeFor(e.ignition, e.throttlePercent);
}

void feedFrame(Link2Monitor& mon, const VehicleState& s, uint32_t nowMs) {
    uint8_t frame[link2::kFrameLen];
    link2::encodeFrame(s, frame);
    for (uint8_t b : frame) {
        mon.feedByte(b, nowMs);
    }
}

int32_t blockPeak(const int16_t* buf, size_t frames) {
    int32_t peak = 0;
    for (size_t i = 0; i < frames * 2; ++i) {
        int32_t v = buf[i] < 0 ? -buf[i] : buf[i];
        if (v > peak) peak = v;
    }
    return peak;
}

} // namespace

void setUp() {}
void tearDown() {}

void test_full_chain_arm_drive_then_link_loss() {
    Link2Monitor mon;
    EngineSim sim;
    EngineSynth synth(soundsynth::EngineSynthConfig{}, 0x55u);
    LightRenderer lights;

    static int16_t audio[256 * 2];
    Rgb px[kNumPixels];

    // --- Phase 1: armed + full throttle for ~1s. Sound should get loud. ---
    VehicleState driving;
    driving.armed = true;
    driving.failsafe = false;
    driving.throttlePercent = 100;
    driving.gear = 3;
    driving.rpm = 5000;

    uint32_t t = 0;
    for (int i = 0; i < 60; ++i) { // 60 control ticks @ ~20ms
        t += 20;
        feedFrame(mon, driving, t);
        mon.poll(t);
        sim.update(t, mon.state());
        const auto& e = sim.engine();
        synth.setParams(e.engineRpm, volumeFor(e), e.ersWhine, e.limiterActive, e.overrunActive);
        // Render a couple of audio blocks per control tick (audio runs faster).
        synth.render(audio, 256);
        synth.render(audio, 256);
        lights.render(mon.state(), mon.status(), t, px);
    }

    TEST_ASSERT_EQUAL(LinkStatus::Up, mon.status());
    TEST_ASSERT_EQUAL(Ignition::Running, sim.engine().ignition);
    TEST_ASSERT_TRUE(sim.engine().engineRpm > 12000);   // near redline
    const int32_t drivingPeak = blockPeak(audio, 256);
    TEST_ASSERT_TRUE(drivingPeak > 3000);               // audibly loud

    // --- Phase 2: link goes silent. After 500ms the monitor projects
    // failsafe; the engine must fall to Off (silent), lights to hazard. ---
    for (int i = 0; i < 60; ++i) {
        t += 20;
        mon.poll(t); // no frames fed
        sim.update(t, mon.state());
        const auto& e = sim.engine();
        synth.setParams(e.engineRpm, volumeFor(e), e.ersWhine, e.limiterActive, e.overrunActive);
        synth.render(audio, 256);
        synth.render(audio, 256);
        lights.render(mon.state(), mon.status(), t, px);
    }

    TEST_ASSERT_EQUAL(LinkStatus::Lost, mon.status());
    TEST_ASSERT_EQUAL(Ignition::Off, sim.engine().ignition);
    // Volume ramped to 0 -> silence.
    synth.render(audio, 256);
    TEST_ASSERT_EQUAL_INT32(0, blockPeak(audio, 256));

    // Lights are in hazard (all amber at the on-phase). Find an on-phase.
    lights.render(mon.state(), mon.status(), 0, px);
    bool amber = px[0].r > 0 && px[0].g > 0 && px[0].b == 0;
    lights.render(mon.state(), mon.status(), 250, px); // off-phase
    bool off = (px[0] == Rgb{0, 0, 0});
    TEST_ASSERT_TRUE(amber && off);
}

void test_full_drive_script_produces_audible_sound() {
    Link2Monitor mon;
    EngineSim sim;
    EngineSynth synth;
    LightRenderer lights;
    static int16_t audio[256 * 2];
    Rgb px[kNumPixels];

    int32_t scriptPeak = 0;
    uint32_t t = 0;
    for (int step = 0; step < 400; ++step) {
        t += 20;
        VehicleState s;
        s.armed = true;
        s.failsafe = false;
        // Sweep throttle, toggle ERS deploy and gear, brake sometimes.
        s.throttlePercent = static_cast<int8_t>((step * 7) % 101);
        s.gear = static_cast<uint8_t>(1 + (step / 40) % 4);
        s.driveMode = 2;
        s.ersDeploying = (step % 5) == 0;
        s.ersPercent = static_cast<uint8_t>((step * 3) % 101);
        s.braking = (step % 11) == 0;
        s.steeringPercent = static_cast<int8_t>(((step * 13) % 200) - 100);
        feedFrame(mon, s, t);
        sim.update(t, mon.state());
        const auto& e = sim.engine();
        synth.setParams(e.engineRpm, volumeFor(e), e.ersWhine, e.limiterActive, e.overrunActive);
        synth.render(audio, 256);
        lights.render(mon.state(), mon.status(), t, px);
        const int32_t p = blockPeak(audio, 256);
        if (p > scriptPeak) scriptPeak = p;
    }
    // The former assertion here (blockPeak <= 32767) was vacuous: blockPeak
    // returns the magnitude of int16 samples, so it can never exceed 32768.
    // The meaningful chain-level invariant is that a full armed drive script
    // actually produces audible engine sound end-to-end (bytes -> monitor ->
    // sim -> synth). Silence would mean the chain broke somewhere upstream.
    TEST_ASSERT_TRUE(scriptPeak > 3000); // audibly loud at some point in the run
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_full_chain_arm_drive_then_link_loss);
    RUN_TEST(test_full_drive_script_produces_audible_sound);
    return UNITY_END();
}
