#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

struct ScanPoint {
  uint16_t distance_mm;
  uint8_t intensity;
  bool valid;
};

uint8_t ld03_crc8(const uint8_t* data, size_t length);

class LD03Driver {
 public:
  static constexpr size_t kFrameLength = 47;
  static constexpr size_t kScanPointCount = 360;

  void ingest(uint8_t byte);
  bool has_new_scan() const { return has_new_scan_; }
  void clear_new_scan() { has_new_scan_ = false; }
  const std::array<ScanPoint, kScanPointCount>& scan() const { return scan_; }

 private:
  void reset_frame();
  void process_frame();

  std::array<uint8_t, kFrameLength> frame_{};
  size_t frame_length_ = 0;
  std::array<ScanPoint, kScanPointCount> scan_{};
  bool has_new_scan_ = false;
};
