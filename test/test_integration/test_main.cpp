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

// One control tick's parameter hand-off, EXACTLY as src/main.cpp does it:
// state volume from the production decision, the link2 v2 operator volume
// composed in, the profile normalized, everything through packParams ->
// applyPackedParams (the real cross-core word). No test-local copy of any
// mapping or constant.
void publishTick(EngineSynth& synth, const enginesim::EngineState& e,
                 const VehicleState& v) {
    synth.applyPackedParams(soundsynth::packParams(
        e.engineRpm,
        audiodecision::applyOperatorVolume(
            audiodecision::synthVolumeFor(e.ignition, e.throttlePercent), v.volume),
        e.ersWhine, e.limiterActive, e.overrunActive,
        audiodecision::normalizeSoundProfile(v.soundProfile)));
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

    // --- Phase 1: armed + full throttle for ~1s, operator volume pinned at
    // the MAXIMUM (100) so phase 2 proves failsafe wins over it. ---
    VehicleState driving;
    driving.armed = true;
    driving.failsafe = false;
    driving.throttlePercent = 100;
    driving.gear = 3;
    driving.rpm = 5000;
    driving.volume = 100; // loudest the operator can ask for

    uint32_t t = 0;
    for (int i = 0; i < 60; ++i) { // 60 control ticks @ ~20ms
        t += 20;
        feedFrame(mon, driving, t);
        mon.poll(t);
        sim.update(t, mon.state());
        publishTick(synth, sim.engine(), mon.state());
        // Render a couple of audio blocks per control tick (audio runs faster).
        synth.render(audio, 256);
        synth.render(audio, 256);
        lights.render(mon.state(), mon.status(), sim.engine().ignition, t, px);
    }

    TEST_ASSERT_EQUAL(LinkStatus::Up, mon.status());
    TEST_ASSERT_EQUAL(Ignition::Running, sim.engine().ignition);
    TEST_ASSERT_TRUE(sim.engine().engineRpm > 12000);   // near redline
    const int32_t drivingPeak = blockPeak(audio, 256);
    TEST_ASSERT_TRUE(drivingPeak > 3000);               // audibly loud

    // --- Phase 2: link goes silent. After 500ms the monitor projects
    // failsafe; the engine must fall to Off (silent), lights to hazard --
    // even though the HELD operator volume is still 100 (failsafe-over-
    // volume precedence, end to end). ---
    for (int i = 0; i < 60; ++i) {
        t += 20;
        mon.poll(t); // no frames fed
        sim.update(t, mon.state());
        publishTick(synth, sim.engine(), mon.state());
        synth.render(audio, 256);
        synth.render(audio, 256);
        lights.render(mon.state(), mon.status(), sim.engine().ignition, t, px);
    }

    TEST_ASSERT_EQUAL(LinkStatus::Lost, mon.status());
    TEST_ASSERT_EQUAL(Ignition::Off, sim.engine().ignition);
    // The monitor HOLDS the operator volume (config, not state) ...
    TEST_ASSERT_EQUAL_UINT8(100, mon.state().volume);
    // ... and the engine is silent anyway: volume never outranks failsafe.
    synth.render(audio, 256);
    TEST_ASSERT_EQUAL_INT32(0, blockPeak(audio, 256));

    // Lights are in hazard (all amber at the on-phase). Find an on-phase.
    lights.render(mon.state(), mon.status(), sim.engine().ignition, 0, px);
    bool amber = px[0].r > 0 && px[0].g > 0 && px[0].b == 0;
    lights.render(mon.state(), mon.status(), sim.engine().ignition, 250, px); // off-phase
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
        // Exercise the v2 fields across the script: alternate the voice
        // profile (incl. a reserved value that must fall back to V10) and
        // sweep the operator volume through its upper half.
        s.soundProfile = static_cast<uint8_t>((step / 100) % 3); // 0, 1, 2(reserved)
        s.volume = static_cast<uint8_t>(50 + (step % 51));       // 50..100
        feedFrame(mon, s, t);
        sim.update(t, mon.state());
        publishTick(synth, sim.engine(), mon.state());
        synth.render(audio, 256);
        lights.render(mon.state(), mon.status(), sim.engine().ignition, t, px);
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

// The v2 operator config rides actual FRAMES to the synth: two identical
// chains that differ only in the transmitted soundProfile byte diverge
// audibly (V6 selected over the wire), a reserved byte behaves exactly like
// V10, and the transmitted volume byte scales the output (quiet < loud, and
// volume 0 converges to true silence) -- all through monitor -> sim ->
// packParams -> applyPackedParams, the production path.
void test_wire_profile_and_volume_reach_the_synth() {
    Link2Monitor monV10;
    Link2Monitor monV6;
    Link2Monitor monReserved;
    EngineSim simV10;
    EngineSim simV6;
    EngineSim simReserved;
    EngineSynth synthV10(soundsynth::EngineSynthConfig{}, 0x42u);
    EngineSynth synthV6(soundsynth::EngineSynthConfig{}, 0x42u);
    EngineSynth synthReserved(soundsynth::EngineSynthConfig{}, 0x42u);

    static int16_t a10[256 * 2];
    static int16_t a6[256 * 2];
    static int16_t ar[256 * 2];

    VehicleState s;
    s.armed = true;
    s.failsafe = false;
    s.throttlePercent = 80;
    s.volume = 100;

    bool differs = false;
    uint32_t t = 0;
    for (int i = 0; i < 60; ++i) {
        t += 20;
        s.soundProfile = 0;
        feedFrame(monV10, s, t);
        s.soundProfile = 1;
        feedFrame(monV6, s, t);
        s.soundProfile = 7; // reserved: must behave exactly like V10
        feedFrame(monReserved, s, t);

        simV10.update(t, monV10.state());
        simV6.update(t, monV6.state());
        simReserved.update(t, monReserved.state());
        publishTick(synthV10, simV10.engine(), monV10.state());
        publishTick(synthV6, simV6.engine(), monV6.state());
        publishTick(synthReserved, simReserved.engine(), monReserved.state());

        synthV10.render(a10, 256);
        synthV6.render(a6, 256);
        synthReserved.render(ar, 256);
        for (int j = 0; j < 256 * 2; ++j) {
            if (a10[j] != a6[j]) differs = true;
            TEST_ASSERT_EQUAL_INT16(a10[j], ar[j]); // reserved == V10, sample-exact
        }
    }
    TEST_ASSERT_TRUE(differs); // the wire byte really selected the V6

    // Volume over the wire: same chain, volume 25 vs 100 -> strictly quieter
    // but still audible; volume 0 -> converges to true silence.
    Link2Monitor monQuiet;
    EngineSim simQuiet;
    EngineSynth synthQuiet(soundsynth::EngineSynthConfig{}, 0x42u);
    s.soundProfile = 0;
    s.volume = 25;
    t = 0;
    for (int i = 0; i < 60; ++i) {
        t += 20;
        feedFrame(monQuiet, s, t);
        simQuiet.update(t, monQuiet.state());
        publishTick(synthQuiet, simQuiet.engine(), monQuiet.state());
        synthQuiet.render(a6, 256); // reuse the buffer
    }
    const int32_t quietPeak = blockPeak(a6, 256);
    const int32_t loudPeak = blockPeak(a10, 256); // the volume-100 V10 run above
    TEST_ASSERT_TRUE(quietPeak > 0);
    TEST_ASSERT_TRUE(quietPeak < loudPeak);

    Link2Monitor monMute;
    EngineSim simMute;
    EngineSynth synthMute(soundsynth::EngineSynthConfig{}, 0x42u);
    s.volume = 0;
    t = 0;
    for (int i = 0; i < 60; ++i) {
        t += 20;
        feedFrame(monMute, s, t);
        simMute.update(t, monMute.state());
        publishTick(synthMute, simMute.engine(), monMute.state());
        synthMute.render(ar, 256);
    }
    TEST_ASSERT_EQUAL_INT32(0, blockPeak(ar, 256)); // engine RUNNING, yet silent
    TEST_ASSERT_EQUAL(Ignition::Running, simMute.engine().ignition);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_full_chain_arm_drive_then_link_loss);
    RUN_TEST(test_full_drive_script_produces_audible_sound);
    RUN_TEST(test_wire_profile_and_volume_reach_the_synth);
    return UNITY_END();
}
