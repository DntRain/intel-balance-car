#include <array>
#include <cassert>
#include <cstdint>

#include "ld03_driver.h"
#include "scan_conversion.h"

namespace {

std::array<uint8_t, LD03Driver::kFrameLength> valid_frame() {
  std::array<uint8_t, LD03Driver::kFrameLength> frame{};
  frame[0] = 0x54;
  frame[1] = 0x2C;
  frame[2] = 0xE8;
  frame[3] = 0x03;
  frame[4] = 0x28;
  frame[5] = 0x23;  // 90.00 degrees

  for (uint8_t index = 0; index < 12; ++index) {
    const uint16_t distance = static_cast<uint16_t>(1000 + index);
    const size_t offset = 6 + index * 3;
    frame[offset] = static_cast<uint8_t>(distance & 0xFF);
    frame[offset + 1] = static_cast<uint8_t>(distance >> 8);
    frame[offset + 2] = static_cast<uint8_t>(20 + index);
  }

  frame[42] = 0x74;
  frame[43] = 0x27;  // 101.00 degrees
  frame[44] = 0;
  frame[45] = 0;
  frame[46] = ld03_crc8(frame.data(), frame.size() - 1);
  return frame;
}

void test_valid_frame_updates_scan() {
  LD03Driver driver;
  const auto frame = valid_frame();
  for (const uint8_t byte : frame) driver.ingest(byte);

  assert(driver.has_new_scan());
  const auto& scan = driver.scan();
  assert(scan[90].valid);
  assert(scan[90].distance_mm == 1000);
  assert(scan[90].intensity == 20);
  assert(scan[101].valid);
  assert(scan[101].distance_mm == 1011);
}

void test_invalid_crc_does_not_update_scan() {
  LD03Driver driver;
  auto frame = valid_frame();
  frame.back() ^= 0xFF;
  for (const uint8_t byte : frame) driver.ingest(byte);

  assert(!driver.has_new_scan());
  assert(!driver.scan()[90].valid);
}

void test_scan_conversion_uses_meters_and_infinity() {
  std::array<ScanPoint, LD03Driver::kScanPointCount> points{};
  points[0] = ScanPoint{1000, 42, true};
  points[1] = ScanPoint{0, 0, false};
  std::array<float, LD03Driver::kScanPointCount> ranges{};
  std::array<float, LD03Driver::kScanPointCount> intensities{};

  fill_scan_ranges(points, ranges, intensities);

  assert(ranges[0] == 1.0F);
  assert(intensities[0] == 42.0F);
  assert(ranges[1] == scan_infinity());
  assert(intensities[1] == 0.0F);
}

}  // namespace

int main() {
  test_valid_frame_updates_scan();
  test_invalid_crc_does_not_update_scan();
  test_scan_conversion_uses_meters_and_infinity();
}
