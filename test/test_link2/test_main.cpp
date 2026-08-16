#include <unity.h>

#include <cstring>

#include "link2/Link2Codec.hpp"

// lib/link2 is copied VERBATIM from w17-control-fw (which owns the protocol).
// These tests pin the receiving-side behaviors this board depends on; the
// golden frame must stay byte-identical to the sender repo's.

using link2::DecodeResult;
using link2::Link2FrameAssembler;
using link2::VehicleState;

namespace {

VehicleState makeGoldenState() {
    VehicleState s;
    s.throttlePercent = 42;
    s.steeringPercent = -25;
    s.braking = false;
    s.drsOpen = true;
    s.armed = true;
    s.failsafe = false;
    s.lowBattery = false;
    s.ersDeploying = true;
    s.gear = 3;
    s.rpm = 1500;
    s.batteryMv = 7900;
    s.ersPercent = 60;
    s.driveMode = 2;
    s.soundProfile = link2::kSoundProfileV6Hybrid; // v2: non-default, pins the byte
    s.volume = 80;                                 // v2: == kDefaultVolume as a value
    return s;
}

// Byte-identical to w17-control-fw's kGoldenFrame (test_link2 there) and to
// the worked example in docs/link2_protocol.md on BOTH repos -- the shared
// witness that the two verbatim codec copies speak the same v2 wire format.
const uint8_t kGoldenFrame[link2::kFrameLen] = {
    0xA5, 0x0D, 0x02, 0x2A, 0xE7, 0x4C, 0x03, 0xDC,
    0x05, 0xDC, 0x1E, 0x3C, 0x02, 0x01, 0x50, 0xCC,
};

} // namespace

void setUp() {}
void tearDown() {}

void test_golden_frame_bytes() {
    uint8_t frame[link2::kFrameLen];
    TEST_ASSERT_EQUAL_UINT32(link2::kFrameLen, link2::encodeFrame(makeGoldenState(), frame));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(kGoldenFrame, frame, link2::kFrameLen);
}

void test_decode_roundtrip() {
    uint8_t frame[link2::kFrameLen];
    link2::encodeFrame(makeGoldenState(), frame);

    VehicleState out;
    TEST_ASSERT_EQUAL(DecodeResult::Ok, link2::decodeFrame(frame, sizeof(frame), out));
    TEST_ASSERT_EQUAL_INT8(42, out.throttlePercent);
    TEST_ASSERT_EQUAL_INT8(-25, out.steeringPercent);
    TEST_ASSERT_TRUE(out.drsOpen);
    TEST_ASSERT_TRUE(out.armed);
    TEST_ASSERT_TRUE(out.ersDeploying);
    TEST_ASSERT_FALSE(out.failsafe);
    TEST_ASSERT_EQUAL_UINT8(3, out.gear);
    TEST_ASSERT_EQUAL_UINT16(1500, out.rpm);
    TEST_ASSERT_EQUAL_UINT16(7900, out.batteryMv);
    TEST_ASSERT_EQUAL_UINT8(60, out.ersPercent);
    TEST_ASSERT_EQUAL_UINT8(2, out.driveMode);
    TEST_ASSERT_EQUAL_UINT8(link2::kSoundProfileV6Hybrid, out.soundProfile);
    TEST_ASSERT_EQUAL_UINT8(80, out.volume);
}

// The v2 sound bytes arrive RAW (like driveMode): reserved profiles and
// volumes past kVolumeMax reach the consumer intact, which then applies the
// documented fallback (>= kSoundProfileCount -> V10) and clamp (>100 -> 100).
// This board's consumer-side rules are pinned in test_audiodecision.
void test_reserved_sound_values_arrive_raw() {
    VehicleState in = makeGoldenState();
    in.soundProfile = link2::kSoundProfileCount; // first reserved value
    in.volume = 101;                             // just past kVolumeMax

    uint8_t frame[link2::kFrameLen];
    link2::encodeFrame(in, frame);
    VehicleState out;
    TEST_ASSERT_EQUAL(DecodeResult::Ok, link2::decodeFrame(frame, sizeof(frame), out));
    TEST_ASSERT_EQUAL_UINT8(link2::kSoundProfileCount, out.soundProfile);
    TEST_ASSERT_EQUAL_UINT8(101, out.volume);
}

// The coordinated-flash rule from the receiver's own seat: yesterday's golden
// v1 frame (length 0x0B, version 1, CRC genuinely valid) must be HARD-
// REJECTED at the length byte. If board #1 is still sending v1, this board
// never decodes a frame and sits in its 500 ms staleness failsafe -- silence
// + hazards -- until BOTH boards are flashed together (docs/link2_protocol.md,
// v1 -> v2).
void test_v1_frame_hard_rejected() {
    const uint8_t v1Frame[14] = {0xA5, 0x0B, 0x01, 0x2A, 0xE7, 0x4C, 0x03,
                                 0xDC, 0x05, 0xDC, 0x1E, 0x3C, 0x02, 0xCE};
    TEST_ASSERT_EQUAL_HEX8(v1Frame[13], link2::computeCrc8(v1Frame + 1, 12)); // well-formed v1

    VehicleState out;
    TEST_ASSERT_EQUAL(DecodeResult::BadLength, link2::decodeFrame(v1Frame, sizeof(v1Frame), out));

    Link2FrameAssembler assembler;
    TEST_ASSERT_EQUAL(Link2FrameAssembler::FeedResult::Incomplete,
                      assembler.feedByte(v1Frame[0]));
    // Rejected AT the length byte -- no v1 body byte is ever buffered.
    TEST_ASSERT_EQUAL(Link2FrameAssembler::FeedResult::FrameInvalid,
                      assembler.feedByte(v1Frame[1]));
}

void test_decode_rejections() {
    uint8_t frame[link2::kFrameLen];
    link2::encodeFrame(makeGoldenState(), frame);
    VehicleState out;

    uint8_t bad[link2::kFrameLen];

    std::memcpy(bad, frame, sizeof(bad));
    bad[0] = 0x00;
    TEST_ASSERT_EQUAL(DecodeResult::BadStart, link2::decodeFrame(bad, sizeof(bad), out));

    std::memcpy(bad, frame, sizeof(bad));
    bad[1] = 0x0A;
    TEST_ASSERT_EQUAL(DecodeResult::BadLength, link2::decodeFrame(bad, sizeof(bad), out));

    std::memcpy(bad, frame, sizeof(bad));
    bad[link2::kFrameLen - 1] ^= 0xFF;
    TEST_ASSERT_EQUAL(DecodeResult::CrcMismatch, link2::decodeFrame(bad, sizeof(bad), out));

    // CRC checked before version: BadVersion means a well-formed newer frame.
    std::memcpy(bad, frame, sizeof(bad));
    bad[2] = 9;
    bad[link2::kFrameLen - 1] = link2::computeCrc8(bad + 1, 1 + link2::kPayloadLen);
    TEST_ASSERT_EQUAL(DecodeResult::BadVersion, link2::decodeFrame(bad, sizeof(bad), out));
}

void test_assembler_hard_rejects_bad_length_byte() {
    Link2FrameAssembler assembler;
    TEST_ASSERT_EQUAL(Link2FrameAssembler::FeedResult::Incomplete,
                      assembler.feedByte(link2::kStartByte));
    // A corrupt 0xFF length is rejected NOW, per the protocol's receiver rule.
    TEST_ASSERT_EQUAL(Link2FrameAssembler::FeedResult::FrameInvalid, assembler.feedByte(0xFF));

    uint8_t frame[link2::kFrameLen];
    link2::encodeFrame(makeGoldenState(), frame);
    Link2FrameAssembler::FeedResult result = Link2FrameAssembler::FeedResult::Incomplete;
    for (uint8_t b : frame) {
        result = assembler.feedByte(b);
    }
    TEST_ASSERT_EQUAL(Link2FrameAssembler::FeedResult::FrameReady, result);
    TEST_ASSERT_EQUAL_INT8(42, assembler.lastState().throttlePercent);
}

void test_assembler_resyncs_after_corruption() {
    uint8_t corrupt[link2::kFrameLen];
    link2::encodeFrame(makeGoldenState(), corrupt);
    corrupt[link2::kFrameLen - 1] ^= 0xFF;

    uint8_t good[link2::kFrameLen];
    link2::encodeFrame(makeGoldenState(), good);

    Link2FrameAssembler assembler;
    for (uint8_t b : corrupt) {
        assembler.feedByte(b);
    }
    Link2FrameAssembler::FeedResult result = Link2FrameAssembler::FeedResult::Incomplete;
    for (uint8_t b : good) {
        result = assembler.feedByte(b);
    }
    TEST_ASSERT_EQUAL(Link2FrameAssembler::FeedResult::FrameReady, result);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_golden_frame_bytes);
    RUN_TEST(test_decode_roundtrip);
    RUN_TEST(test_reserved_sound_values_arrive_raw);
    RUN_TEST(test_v1_frame_hard_rejected);
    RUN_TEST(test_decode_rejections);
    RUN_TEST(test_assembler_hard_rejects_bad_length_byte);
    RUN_TEST(test_assembler_resyncs_after_corruption);
    return UNITY_END();
}
