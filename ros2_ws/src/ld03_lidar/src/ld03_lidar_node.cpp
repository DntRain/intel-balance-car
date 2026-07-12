#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "ld03_lidar/ld03_parser.hpp"

namespace ld03_lidar {
namespace {

constexpr size_t kScanPointCount = 360;
constexpr float kPi = 3.14159265358979323846F;
constexpr float kRangeMinMetres = 0.02F;
constexpr float kRangeMaxMetres = 12.0F;

speed_t baud_constant(int baud_rate) {
  switch (baud_rate) {
    case 115200:
      return B115200;
    default:
      return 0;
  }
}

}  // namespace

class LD03LidarNode final : public rclcpp::Node {
 public:
  LD03LidarNode()
      : Node("ld03_lidar_node"),
        port_(declare_parameter<std::string>("port", "/dev/ttyS1")),
        baud_rate_(declare_parameter<int>("baud_rate", 115200)),
        frame_id_(declare_parameter<std::string>("frame_id", "laser_link")),
        scan_topic_(declare_parameter<std::string>("scan_topic", "/scan")) {
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(5),
                               std::bind(&LD03LidarNode::read_serial, this));
  }

  ~LD03LidarNode() override {
    close_serial();
  }

 private:
  bool open_serial() {
    const speed_t baud = baud_constant(baud_rate_);
    if (baud == 0) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Unsupported baud_rate=%d; only 115200 is supported", baud_rate_);
      return false;
    }

    const int descriptor = open(port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (descriptor < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Cannot open %s: %s", port_.c_str(), std::strerror(errno));
      return false;
    }

    termios options{};
    if (tcgetattr(descriptor, &options) != 0) {
      RCLCPP_WARN(get_logger(), "Cannot read serial settings for %s: %s",
                  port_.c_str(), std::strerror(errno));
      close(descriptor);
      return false;
    }

    cfmakeraw(&options);
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    if (tcsetattr(descriptor, TCSANOW, &options) != 0) {
      RCLCPP_WARN(get_logger(), "Cannot configure %s: %s",
                  port_.c_str(), std::strerror(errno));
      close(descriptor);
      return false;
    }

    serial_fd_ = descriptor;
    RCLCPP_INFO(get_logger(), "Reading LD03 data from %s at %d baud",
                port_.c_str(), baud_rate_);
    return true;
  }

  void close_serial() {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  void read_serial() {
    if (serial_fd_ < 0 && !open_serial()) {
      return;
    }

    std::array<uint8_t, 1024> buffer{};
    const ssize_t bytes_read = read(serial_fd_, buffer.data(), buffer.size());
    if (bytes_read < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_WARN(get_logger(), "Read failed for %s: %s; reconnecting",
                    port_.c_str(), std::strerror(errno));
        close_serial();
      }
      report_diagnostics();
      return;
    }

    received_byte_count_ += static_cast<uint64_t>(bytes_read);
    for (ssize_t index = 0; index < bytes_read; ++index) {
      parser_.ingest(buffer[static_cast<size_t>(index)]);
    }

    while (const auto scan = parser_.take_completed_scan()) {
      publish_scan(*scan);
    }
    report_diagnostics();
  }

  void report_diagnostics() {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "LD03 diagnostics: bytes=%llu valid_frames=%llu crc_errors=%llu completed_scans=%llu",
        static_cast<unsigned long long>(received_byte_count_),
        static_cast<unsigned long long>(parser_.valid_frame_count()),
        static_cast<unsigned long long>(parser_.invalid_crc_count()),
        static_cast<unsigned long long>(parser_.completed_scan_count()));
  }

  void publish_scan(const Scan& scan) {
    const auto now = this->now();
    sensor_msgs::msg::LaserScan message;
    message.header.stamp = now;
    message.header.frame_id = frame_id_;
    message.angle_min = 0.0F;
    message.angle_max = 2.0F * kPi;
    message.angle_increment = (2.0F * kPi) / static_cast<float>(kScanPointCount);
    message.range_min = kRangeMinMetres;
    message.range_max = kRangeMaxMetres;
    message.ranges.resize(kScanPointCount, std::numeric_limits<float>::infinity());
    message.intensities.resize(kScanPointCount, 0.0F);

    if (last_scan_stamp_.nanoseconds() != 0) {
      message.scan_time = static_cast<float>((now - last_scan_stamp_).seconds());
      message.time_increment = message.scan_time / static_cast<float>(kScanPointCount);
    }
    last_scan_stamp_ = now;

    for (size_t index = 0; index < scan.points.size(); ++index) {
      const ScanPoint& point = scan.points[index];
      const float range_metres = static_cast<float>(point.distance_mm) / 1000.0F;
      if (point.valid && range_metres >= kRangeMinMetres && range_metres <= kRangeMaxMetres) {
        message.ranges[index] = range_metres;
        message.intensities[index] = static_cast<float>(point.intensity);
      }
    }

    publisher_->publish(message);
  }

  std::string port_;
  int baud_rate_;
  std::string frame_id_;
  std::string scan_topic_;
  int serial_fd_ = -1;
  uint64_t received_byte_count_ = 0;
  LD03Parser parser_;
  rclcpp::Time last_scan_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ld03_lidar

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ld03_lidar::LD03LidarNode>());
  rclcpp::shutdown();
  return 0;
}
