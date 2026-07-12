#include "ld03_lidar/ld03_parser.hpp"

namespace ld03_lidar {
namespace {

constexpr uint8_t kHeader = 0x54;
constexpr uint8_t kVersionLength = 0x2C;
constexpr size_t kPointsPerFrame = 12;

uint16_t read_u16_le(const uint8_t* data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

}  // namespace

uint8_t crc8(const uint8_t* data, size_t length) {
  uint8_t crc = 0;
  for (size_t index = 0; index < length; ++index) {
    crc ^= data[index];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80U) ? static_cast<uint8_t>((crc << 1U) ^ 0x4DU)
                           : static_cast<uint8_t>(crc << 1U);
    }
  }
  return crc;
}

void LD03Parser::ingest(uint8_t byte) {
  if (frame_length_ == 0) {
    if (byte == kHeader) {
      frame_[frame_length_++] = byte;
    }
    return;
  }

  if (frame_length_ == 1) {
    if (byte == kVersionLength) {
      frame_[frame_length_++] = byte;
    } else {
      reset_frame();
      if (byte == kHeader) {
        frame_[frame_length_++] = byte;
      }
    }
    return;
  }

  frame_[frame_length_++] = byte;
  if (frame_length_ == kFrameLength) {
    process_frame();
    reset_frame();
  }
}

std::optional<Scan> LD03Parser::take_completed_scan() {
  std::optional<Scan> result = completed_scan_;
  completed_scan_.reset();
  return result;
}

void LD03Parser::reset_frame() {
  frame_length_ = 0;
}

void LD03Parser::clear_active_scan() {
  active_scan_ = Scan{};
}

void LD03Parser::process_frame() {
  if (crc8(frame_.data(), kFrameLength - 1) != frame_.back()) {
    ++invalid_crc_count_;
    return;
  }
  ++valid_frame_count_;

  const int32_t start_angle = read_u16_le(frame_.data() + 4);
  int32_t end_angle = read_u16_le(frame_.data() + 42);
  if (end_angle < start_angle) {
    end_angle += 36000;
  }

  bool wrapped_this_frame = false;
  for (size_t index = 0; index < kPointsPerFrame; ++index) {
    const int32_t angle = start_angle +
                          ((end_angle - start_angle) * static_cast<int32_t>(index)) /
                              static_cast<int32_t>(kPointsPerFrame - 1);
    if (angle >= 36000 && !wrapped_this_frame) {
      if (synchronized_to_wrap_) {
        completed_scan_ = active_scan_;
        ++completed_scan_count_;
      }
      clear_active_scan();
      synchronized_to_wrap_ = true;
      wrapped_this_frame = true;
    }

    const size_t point_offset = 6 + index * 3;
    const uint16_t distance_mm = read_u16_le(frame_.data() + point_offset);
    if (distance_mm == 0) {
      continue;
    }

    const size_t degree = static_cast<size_t>((angle % 36000) / 100);
    active_scan_.points[degree] = ScanPoint{
        distance_mm,
        frame_[point_offset + 2],
        true,
    };
  }
}

}  // namespace ld03_lidar
