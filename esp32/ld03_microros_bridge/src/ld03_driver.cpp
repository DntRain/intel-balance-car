#include "ld03_driver.h"

namespace {
constexpr uint8_t kHeader = 0x54;
constexpr uint8_t kVersionLength = 0x2C;
constexpr size_t kPointsPerFrame = 12;

uint16_t read_u16_le(const uint8_t* data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}
}  // namespace

uint8_t ld03_crc8(const uint8_t* data, size_t length) {
  uint8_t crc = 0;
  for (size_t index = 0; index < length; ++index) {
    crc ^= data[index];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x4D)
                          : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

void LD03Driver::ingest(uint8_t byte) {
  if (frame_length_ == 0) {
    if (byte == kHeader) frame_[frame_length_++] = byte;
    return;
  }

  if (frame_length_ == 1) {
    if (byte == kVersionLength) {
      frame_[frame_length_++] = byte;
    } else {
      reset_frame();
      if (byte == kHeader) frame_[frame_length_++] = byte;
    }
    return;
  }

  frame_[frame_length_++] = byte;
  if (frame_length_ != kFrameLength) return;

  process_frame();
  reset_frame();
}

void LD03Driver::reset_frame() {
  frame_length_ = 0;
}

void LD03Driver::process_frame() {
  if (ld03_crc8(frame_.data(), kFrameLength - 1) != frame_.back()) return;

  int32_t start_angle = read_u16_le(frame_.data() + 4);
  int32_t end_angle = read_u16_le(frame_.data() + 42);
  if (end_angle < start_angle) end_angle += 36000;
  const int32_t angle_step = (end_angle - start_angle) /
                             static_cast<int32_t>(kPointsPerFrame - 1);

  for (size_t index = 0; index < kPointsPerFrame; ++index) {
    const size_t point_offset = 6 + index * 3;
    const uint16_t distance = read_u16_le(frame_.data() + point_offset);
    if (distance == 0) continue;

    const int32_t centi_degrees = (start_angle + angle_step * index) % 36000;
    const size_t degrees = static_cast<size_t>(centi_degrees / 100);
    scan_[degrees] = ScanPoint{distance, frame_[point_offset + 2], true};
  }

  has_new_scan_ = true;
}
