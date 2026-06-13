#include "care_hw_bridge/serial_port.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace care_hw_bridge
{
namespace
{

int16_t clamp_to_i16(double value)
{
  const double rounded = std::round(value);
  const double clamped = std::clamp(
    rounded,
    static_cast<double>(std::numeric_limits<int16_t>::min()),
    static_cast<double>(std::numeric_limits<int16_t>::max()));
  return static_cast<int16_t>(clamped);
}

void push_i16_le(std::vector<uint8_t> & packet, int16_t value)
{
  const auto raw = static_cast<uint16_t>(value);
  packet.push_back(static_cast<uint8_t>(raw & 0xff));
  packet.push_back(static_cast<uint8_t>((raw >> 8) & 0xff));
}

uint8_t xor_checksum(const std::vector<uint8_t> & packet)
{
  uint8_t checksum = 0;
  for (const auto byte : packet) {
    checksum ^= byte;
  }
  return checksum;
}

std::string bytes_to_hex_line(const std::vector<uint8_t> & packet)
{
  constexpr std::array<char, 16> hex_chars {
    '0', '1', '2', '3', '4', '5', '6', '7',
    '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

  std::string line;
  line.reserve(packet.size() * 2 + 1);
  for (const auto byte : packet) {
    line.push_back(hex_chars[(byte >> 4) & 0x0f]);
    line.push_back(hex_chars[byte & 0x0f]);
  }
  line.push_back('\n');
  return line;
}

}  // namespace

class CmdVelSerialBridge : public rclcpp::Node
{
public:
  CmdVelSerialBridge()
  : Node("cmd_vel_serial_bridge"),
    seq_(0),
    last_cmd_time_(this->now())
  {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    baudrate_ = declare_parameter<int>("baudrate", 115200);
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    protocol_ = declare_parameter<std::string>("protocol", "hex");
    wheel_separation_m_ = declare_parameter<double>("wheel_separation_m", 0.458);
    wheel_radius_m_ = declare_parameter<double>("wheel_radius_m", 0.06);
    max_wheel_rad_s_ = declare_parameter<double>("max_wheel_rad_s", 20.0);
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.5);
    send_rate_hz_ = declare_parameter<double>("send_rate_hz", 20.0);
    open_serial_ = declare_parameter<bool>("open_serial", true);
    log_packets_ = declare_parameter<bool>("log_packets", false);

    validate_parameters();

    if (open_serial_) {
      serial_.open(serial_port_, baudrate_);
      RCLCPP_INFO(
        get_logger(), "Opened %s at %d baud; protocol=%s",
        serial_port_.c_str(), baudrate_, protocol_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Serial output disabled by open_serial=false");
    }

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        latest_cmd_ = *msg;
        last_cmd_time_ = now();
      });

    const auto period = std::chrono::duration<double>(1.0 / send_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CmdVelSerialBridge::send_command, this));
  }

private:
  void validate_parameters() const
  {
    if (wheel_separation_m_ <= 0.0) {
      throw std::runtime_error("wheel_separation_m must be positive");
    }
    if (wheel_radius_m_ <= 0.0) {
      throw std::runtime_error("wheel_radius_m must be positive");
    }
    if (max_wheel_rad_s_ <= 0.0) {
      throw std::runtime_error("max_wheel_rad_s must be positive");
    }
    if (command_timeout_s_ <= 0.0) {
      throw std::runtime_error("command_timeout_s must be positive");
    }
    if (send_rate_hz_ <= 0.0) {
      throw std::runtime_error("send_rate_hz must be positive");
    }
    if (protocol_ != "hex" && protocol_ != "binary" && protocol_ != "ascii") {
      throw std::runtime_error("protocol must be one of: hex, binary, ascii");
    }
  }

  void send_command()
  {
    geometry_msgs::msg::Twist cmd = latest_cmd_;
    const double age_s = (now() - last_cmd_time_).seconds();
    if (age_s > command_timeout_s_) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    }

    const double half_track = wheel_separation_m_ * 0.5;
    double left_rad_s = (cmd.linear.x - cmd.angular.z * half_track) / wheel_radius_m_;
    double right_rad_s = (cmd.linear.x + cmd.angular.z * half_track) / wheel_radius_m_;

    left_rad_s = std::clamp(left_rad_s, -max_wheel_rad_s_, max_wheel_rad_s_);
    right_rad_s = std::clamp(right_rad_s, -max_wheel_rad_s_, max_wheel_rad_s_);

    try {
      if (protocol_ == "ascii") {
        send_ascii(left_rad_s, right_rad_s);
      } else {
        const auto packet = make_binary_packet(left_rad_s, right_rad_s);
        if (protocol_ == "hex") {
          send_hex(packet);
        } else {
          send_binary(packet);
        }
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Failed to send wheel command: %s", ex.what());
    }
  }

  std::vector<uint8_t> make_binary_packet(double left_rad_s, double right_rad_s)
  {
    const int16_t left_mrad_s = clamp_to_i16(left_rad_s * 1000.0);
    const int16_t right_mrad_s = clamp_to_i16(right_rad_s * 1000.0);

    std::vector<uint8_t> packet;
    packet.reserve(8);
    packet.push_back(0xaa);
    packet.push_back(0x55);
    packet.push_back(seq_++);
    push_i16_le(packet, left_mrad_s);
    push_i16_le(packet, right_mrad_s);
    packet.push_back(xor_checksum(packet));
    return packet;
  }

  void send_hex(const std::vector<uint8_t> & packet)
  {
    const auto line = bytes_to_hex_line(packet);
    if (log_packets_) {
      RCLCPP_INFO(get_logger(), "TX hex: %s", line.c_str());
    }
    if (open_serial_) {
      serial_.write_all(line);
    }
  }

  void send_binary(const std::vector<uint8_t> & packet)
  {
    if (log_packets_) {
      RCLCPP_INFO(get_logger(), "TX binary packet bytes=%zu", packet.size());
    }
    if (open_serial_) {
      serial_.write_all(packet);
    }
  }

  void send_ascii(double left_rad_s, double right_rad_s)
  {
    char buffer[80];
    const int size = std::snprintf(
      buffer, sizeof(buffer), "V %.3f %.3f\n", left_rad_s, right_rad_s);
    if (size < 0) {
      throw std::runtime_error("failed to format ascii packet");
    }

    const std::string line(buffer, static_cast<std::size_t>(size));
    if (log_packets_) {
      RCLCPP_INFO(get_logger(), "TX ascii: %s", line.c_str());
    }
    if (open_serial_) {
      serial_.write_all(line);
    }
  }

  SerialPort serial_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string serial_port_;
  int baudrate_;
  std::string cmd_vel_topic_;
  std::string protocol_;
  double wheel_separation_m_;
  double wheel_radius_m_;
  double max_wheel_rad_s_;
  double command_timeout_s_;
  double send_rate_hz_;
  bool open_serial_;
  bool log_packets_;

  uint8_t seq_;
  geometry_msgs::msg::Twist latest_cmd_;
  rclcpp::Time last_cmd_time_;
};

}  // namespace care_hw_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<care_hw_bridge::CmdVelSerialBridge>());
  rclcpp::shutdown();
  return 0;
}
