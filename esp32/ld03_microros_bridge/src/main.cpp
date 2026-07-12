#if __has_include("network_config.h")
#include "network_config.h"
#else
#error "Copy include/network_config.example.h to include/network_config.h and fill local network values."
#endif

#include <Arduino.h>
#include <WiFi.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/laser_scan.h>

#include "ld03_driver.h"
#include "scan_conversion.h"

namespace {
constexpr uint32_t kLidarBaud = 115200;
constexpr int kLidarRxPin = 16;
constexpr int kLidarTxPin = 17;
constexpr uint32_t kScanIntervalMs = 100;
constexpr uint32_t kReconnectIntervalMs = 5000;
constexpr char kFrameId[] = "laser";

HardwareSerial lidar_uart(1);
LD03Driver lidar;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t scan_publisher;
sensor_msgs__msg__LaserScan scan_message;
std::array<float, LD03Driver::kScanPointCount> ranges;
std::array<float, LD03Driver::kScanPointCount> intensities;
bool microros_ready = false;
uint32_t last_scan_publish_ms = 0;
uint32_t last_connect_attempt_ms = 0;

void connect_wifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.printf("[wifi] connecting to %s\n", WIFI_SSID);
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

bool initialise_microros() {
  IPAddress agent_ip;
  if (!agent_ip.fromString(AGENT_IP)) {
    Serial.printf("[micro-ROS] invalid agent IP: %s\n", AGENT_IP);
    return false;
  }

  set_microros_wifi_transports(const_cast<char*>(WIFI_SSID),
                               const_cast<char*>(WIFI_PASSWORD), agent_ip, AGENT_PORT);
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "ld03_microros_bridge", "", &support) != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(
          &scan_publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan") != RCL_RET_OK) {
    return false;
  }

  scan_message.header.frame_id.data = const_cast<char*>(kFrameId);
  scan_message.header.frame_id.size = sizeof(kFrameId) - 1;
  scan_message.header.frame_id.capacity = sizeof(kFrameId);
  scan_message.angle_min = 0.0F;
  scan_message.angle_max = 2.0F * PI;
  scan_message.angle_increment = (2.0F * PI) / LD03Driver::kScanPointCount;
  scan_message.time_increment = 0.0F;
  scan_message.scan_time = 0.1F;
  scan_message.range_min = 0.02F;
  scan_message.range_max = 12.0F;
  scan_message.ranges.data = ranges.data();
  scan_message.ranges.size = ranges.size();
  scan_message.ranges.capacity = ranges.size();
  scan_message.intensities.data = intensities.data();
  scan_message.intensities.size = intensities.size();
  scan_message.intensities.capacity = intensities.size();
  return true;
}

void publish_scan(uint32_t now_ms) {
  fill_scan_ranges(lidar.scan(), ranges, intensities);
  scan_message.header.stamp.sec = now_ms / 1000;
  scan_message.header.stamp.nanosec = (now_ms % 1000) * 1000000UL;
  if (rcl_publish(&scan_publisher, &scan_message, nullptr) != RCL_RET_OK) {
    Serial.println("[micro-ROS] failed to publish /scan");
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);
  lidar_uart.begin(kLidarBaud, SERIAL_8N1, kLidarRxPin, kLidarTxPin);
  WiFi.mode(WIFI_STA);
  connect_wifi();
  Serial.println("[ld03] bridge started");
}

void loop() {
  while (lidar_uart.available()) {
    lidar.ingest(static_cast<uint8_t>(lidar_uart.read()));
  }

  const uint32_t now_ms = millis();
  if (WiFi.status() != WL_CONNECTED) {
    if (now_ms - last_connect_attempt_ms >= kReconnectIntervalMs) {
      last_connect_attempt_ms = now_ms;
      connect_wifi();
    }
    delay(1);
    return;
  }

  if (!microros_ready && now_ms - last_connect_attempt_ms >= kReconnectIntervalMs) {
    last_connect_attempt_ms = now_ms;
    microros_ready = initialise_microros();
    Serial.printf("[micro-ROS] %s\n", microros_ready ? "connected" : "agent unavailable");
  }

  if (microros_ready && lidar.has_new_scan() && now_ms - last_scan_publish_ms >= kScanIntervalMs) {
    lidar.clear_new_scan();
    publish_scan(now_ms);
    last_scan_publish_ms = now_ms;
  }
  delay(1);
}
