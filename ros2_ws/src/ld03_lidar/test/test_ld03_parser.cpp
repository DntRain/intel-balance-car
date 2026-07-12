#include <array>
#include <cstdint>

#include <gtest/gtest.h>

#include "ld03_lidar/ld03_parser.hpp"

namespace {

std::array<uint8_t, 47> make_frame(uint16_t start_angle,
                                   uint16_t end_angle,
                                   uint16_t distance_mm) {
  std::array<uint8_t, 47> frame{};
  frame[0] = 0x54;
  frame[1] = 0x2C;
  frame[4] = static_cast<uint8_t>(start_angle & 0xFF);
  frame[5] = static_cast<uint8_t>(start_angle >> 8);
  for (size_t point = 0; point < 12; ++point) {
    const size_t offset = 6 + point * 3;
    frame[offset] = static_cast<uint8_t>(distance_mm & 0xFF);
    frame[offset + 1] = static_cast<uint8_t>(distance_mm >> 8);
    frame[offset + 2] = 42;
  }
  frame[42] = static_cast<uint8_t>(end_angle & 0xFF);
  frame[43] = static_cast<uint8_t>(end_angle >> 8);
  frame[46] = ld03_lidar::crc8(frame.data(), frame.size() - 1);
  return frame;
}

void feed_frame(ld03_lidar::LD03Parser& parser,
                const std::array<uint8_t, 47>& frame) {
  for (uint8_t byte : frame) {
    parser.ingest(byte);
  }
}

}  // namespace

TEST(LD03Parser, PublishesCompletedScanAfterAngleWrap) {
  ld03_lidar::LD03Parser parser;

  feed_frame(parser, make_frame(35900, 100, 1000));
  EXPECT_FALSE(parser.take_completed_scan().has_value());
  EXPECT_EQ(parser.valid_frame_count(), 1U);
  EXPECT_EQ(parser.invalid_crc_count(), 0U);

  feed_frame(parser, make_frame(100, 35900, 1200));
  feed_frame(parser, make_frame(35900, 100, 1100));
  const auto scan = parser.take_completed_scan();

  ASSERT_TRUE(scan.has_value());
  EXPECT_EQ(parser.completed_scan_count(), 1U);
  EXPECT_TRUE(scan->points[359].valid);
  EXPECT_TRUE(scan->points[0].valid);
  EXPECT_EQ(scan->points[359].distance_mm, 1100);
  EXPECT_EQ(scan->points[0].distance_mm, 1000);
}

TEST(LD03Parser, RejectsFrameWithInvalidCrc) {
  ld03_lidar::LD03Parser parser;
  auto frame = make_frame(0, 1100, 1000);
  frame.back() ^= 0x01;

  feed_frame(parser, frame);

  EXPECT_FALSE(parser.take_completed_scan().has_value());
  EXPECT_EQ(parser.valid_frame_count(), 0U);
  EXPECT_EQ(parser.invalid_crc_count(), 1U);
}
