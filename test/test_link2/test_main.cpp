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
    return s;
}

const uint8_t kGoldenFrame[link2::kFrameLen] = {
    0xA5, 0x0B, 0x01, 0x2A, 0xE7, 0x4C, 0x03, 0xDC, 0x05, 0xDC, 0x1E, 0x3C, 0x02, 0xCE,
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
    RUN_TEST(test_decode_rejections);
    RUN_TEST(test_assembler_hard_rejects_bad_length_byte);
    RUN_TEST(test_assembler_resyncs_after_corruption);
    return UNITY_END();
}
