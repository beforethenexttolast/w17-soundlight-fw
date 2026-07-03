#pragma once

#include "link2/Link2Frame.hpp"

namespace link2 {

// CRC8 poly 0xD5 (bit-by-bit, MSB-first) -- the same algorithm CRSF uses,
// DUPLICATED here deliberately: lib/link2 must be liftable wholesale into the
// board-#2 project with no dependency on lib/crsf. A test cross-checks the
// two implementations against each other.
uint8_t computeCrc8(const uint8_t* data, size_t len);

// Encodes a complete v1 frame into out[kFrameLen]. Returns kFrameLen.
size_t encodeFrame(const VehicleState& state, uint8_t out[kFrameLen]);

// Decodes a complete frame buffer. Validation order: start -> length -> CRC
// -> version, so a corrupted version byte reports CrcMismatch and BadVersion
// means exactly "well-formed frame from a newer protocol". On any failure,
// `out` is left untouched.
DecodeResult decodeFrame(const uint8_t* data, size_t len, VehicleState& out);

// Byte-stream framer for the receiving side (board #2) and tests. Mirrors
// crsf::CrsfFrameAssembler: resyncs on the next start byte after any failure.
// The length byte is hard-rejected the moment it arrives if it isn't a
// supported v1 length -- otherwise one corrupt 0xFF length byte would swallow
// ~1s of following frames before resync.
class Link2FrameAssembler {
public:
    enum class FeedResult : uint8_t { Incomplete, FrameReady, FrameInvalid };

    FeedResult feedByte(uint8_t b);

    // Valid only immediately after feedByte() returned FrameReady.
    const VehicleState& lastState() const { return lastState_; }

private:
    enum class State : uint8_t { WaitingForStart, ReadingLength, ReadingBody };

    State state_ = State::WaitingForStart;
    uint8_t buffer_[kFrameLen] = {};
    size_t bufferLen_ = 0;
    VehicleState lastState_{};
};

} // namespace link2
