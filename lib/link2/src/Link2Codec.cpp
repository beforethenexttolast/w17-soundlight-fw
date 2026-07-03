#include "link2/Link2Codec.hpp"

namespace link2 {

uint8_t computeCrc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5)
                                : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

size_t encodeFrame(const VehicleState& state, uint8_t out[kFrameLen]) {
    out[0] = kStartByte;
    out[1] = kPayloadLen;
    out[2] = kProtocolVersion;
    out[3] = static_cast<uint8_t>(state.throttlePercent);
    out[4] = static_cast<uint8_t>(state.steeringPercent);
    uint8_t flags = 0;
    if (state.braking) flags |= kFlagBraking;
    if (state.reverse) flags |= kFlagReverse;
    if (state.drsOpen) flags |= kFlagDrsOpen;
    if (state.armed) flags |= kFlagArmed;
    if (state.failsafe) flags |= kFlagFailsafe;
    if (state.lowBattery) flags |= kFlagLowBattery;
    if (state.ersDeploying) flags |= kFlagErsDeploying;
    out[5] = flags;
    out[6] = state.gear;
    out[7] = static_cast<uint8_t>(state.rpm & 0xFF);
    out[8] = static_cast<uint8_t>(state.rpm >> 8);
    out[9] = static_cast<uint8_t>(state.batteryMv & 0xFF);
    out[10] = static_cast<uint8_t>(state.batteryMv >> 8);
    out[11] = state.ersPercent;
    out[12] = state.driveMode;
    out[13] = computeCrc8(out + 1, 1 + kPayloadLen); // over [length + payload]
    return kFrameLen;
}

DecodeResult decodeFrame(const uint8_t* data, size_t len, VehicleState& out) {
    if (len < 4) {
        return DecodeResult::BadLength;
    }
    if (data[0] != kStartByte) {
        return DecodeResult::BadStart;
    }
    if (data[1] != kPayloadLen || len != kFrameLen) {
        return DecodeResult::BadLength;
    }
    const uint8_t receivedCrc = data[kFrameLen - 1];
    if (computeCrc8(data + 1, 1 + kPayloadLen) != receivedCrc) {
        return DecodeResult::CrcMismatch;
    }
    if (data[2] != kProtocolVersion) {
        return DecodeResult::BadVersion; // well-formed, just newer than us
    }

    out.throttlePercent = static_cast<int8_t>(data[3]);
    out.steeringPercent = static_cast<int8_t>(data[4]);
    const uint8_t flags = data[5];
    out.braking = (flags & kFlagBraking) != 0;
    out.reverse = (flags & kFlagReverse) != 0;
    out.drsOpen = (flags & kFlagDrsOpen) != 0;
    out.armed = (flags & kFlagArmed) != 0;
    out.failsafe = (flags & kFlagFailsafe) != 0;
    out.lowBattery = (flags & kFlagLowBattery) != 0;
    out.ersDeploying = (flags & kFlagErsDeploying) != 0;
    out.gear = data[6];
    out.rpm = static_cast<uint16_t>(data[7] | (data[8] << 8));
    out.batteryMv = static_cast<uint16_t>(data[9] | (data[10] << 8));
    out.ersPercent = data[11];
    out.driveMode = data[12];
    return DecodeResult::Ok;
}

Link2FrameAssembler::FeedResult Link2FrameAssembler::feedByte(uint8_t b) {
    switch (state_) {
        case State::WaitingForStart:
            if (b != kStartByte) {
                return FeedResult::Incomplete;
            }
            buffer_[0] = b;
            bufferLen_ = 1;
            state_ = State::ReadingLength;
            return FeedResult::Incomplete;

        case State::ReadingLength:
            if (b != kPayloadLen) {
                // Unsupported length: reject NOW rather than swallowing up to
                // 255 bytes of following frames before resync.
                state_ = State::WaitingForStart;
                bufferLen_ = 0;
                return FeedResult::FrameInvalid;
            }
            buffer_[1] = b;
            bufferLen_ = 2;
            state_ = State::ReadingBody;
            return FeedResult::Incomplete;

        case State::ReadingBody: {
            buffer_[bufferLen_++] = b;
            if (bufferLen_ < kFrameLen) {
                return FeedResult::Incomplete;
            }
            const DecodeResult result = decodeFrame(buffer_, bufferLen_, lastState_);
            state_ = State::WaitingForStart;
            bufferLen_ = 0;
            return result == DecodeResult::Ok ? FeedResult::FrameReady : FeedResult::FrameInvalid;
        }
    }
    return FeedResult::Incomplete; // unreachable
}

} // namespace link2
