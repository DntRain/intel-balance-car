#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace ld03_lidar {

struct ScanPoint {
  uint16_t distance_mm = 0;
  uint8_t intensity = 0;
  bool valid = false;
};

struct Scan {
  std::array<ScanPoint, 360> points{};
};

uint8_t crc8(const uint8_t* data, size_t length);

class LD03Parser {
 public:
  void ingest(uint8_t byte);
  std::optional<Scan> take_completed_scan();
  uint64_t valid_frame_count() const { return valid_frame_count_; }
  uint64_t invalid_crc_count() const { return invalid_crc_count_; }
  uint64_t completed_scan_count() const { return completed_scan_count_; }

 private:
  static constexpr size_t kFrameLength = 47;

  void reset_frame();
  void process_frame();
  void clear_active_scan();

  std::array<uint8_t, kFrameLength> frame_{};
  size_t frame_length_ = 0;
  Scan active_scan_{};
  std::optional<Scan> completed_scan_;
  bool synchronized_to_wrap_ = false;
  uint64_t valid_frame_count_ = 0;
  uint64_t invalid_crc_count_ = 0;
  uint64_t completed_scan_count_ = 0;
};

}  // namespace ld03_lidar
