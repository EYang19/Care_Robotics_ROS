#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace care_hw_bridge
{
namespace
{

constexpr int kTofGridSize = 4;
constexpr int kTofPointCount = kTofGridSize * kTofGridSize;

speed_t baud_to_termios(int baudrate)
{
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      throw std::runtime_error("unsupported baudrate: " + std::to_string(baudrate));
  }
}

double deg_to_rad(double degrees)
{
  return degrees * M_PI / 180.0;
}

std::string normalize_sensor_name(const std::string & name)
{
  if (name == "fl" || name == "left") {
    return "front_left";
  }
  if (name == "fc" || name == "center" || name == "centre") {
    return "front_center";
  }
  if (name == "fr" || name == "right") {
    return "front_right";
  }
  return name;
}

}  // namespace

class FullDuplexSerial
{
public:
  FullDuplexSerial()
  : fd_(-1)
  {
  }

  ~FullDuplexSerial()
  {
    close();
  }

  void open(const std::string & device, int baudrate)
  {
    close();

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error(
        "failed to open " + device + ": " + std::strerror(errno));
    }

    termios tty {};
    if (tcgetattr(fd_, &tty) != 0) {
      const auto error = std::string("tcgetattr failed: ") + std::strerror(errno);
      close();
      throw std::runtime_error(error);
    }

    cfmakeraw(&tty);
    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    const speed_t speed = baud_to_termios(baudrate);
    if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0) {
      const auto error = std::string("failed to set baudrate: ") + std::strerror(errno);
      close();
      throw std::runtime_error(error);
    }

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      const auto error = std::string("tcsetattr failed: ") + std::strerror(errno);
      close();
      throw std::runtime_error(error);
    }

    tcflush(fd_, TCIOFLUSH);
  }

  void close()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  void write_all(const std::string & data)
  {
    if (fd_ < 0) {
      throw std::runtime_error("serial port is not open");
    }

    std::size_t written = 0;
    while (written < data.size()) {
      const ssize_t result = ::write(fd_, data.data() + written, data.size() - written);
      if (result > 0) {
        written += static_cast<std::size_t>(result);
        continue;
      }
      if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
        usleep(1000);
        continue;
      }
      throw std::runtime_error(std::string("serial write failed: ") + std::strerror(errno));
    }
  }

  std::vector<std::string> read_available_lines()
  {
    std::vector<std::string> lines;
    if (fd_ < 0) {
      return lines;
    }

    std::array<char, 256> buffer {};
    while (true) {
      const ssize_t count = ::read(fd_, buffer.data(), buffer.size());
      if (count > 0) {
        for (ssize_t i = 0; i < count; ++i) {
          const char c = buffer[static_cast<std::size_t>(i)];
          if (c == '\n') {
            if (!line_buffer_.empty() && line_buffer_.back() == '\r') {
              line_buffer_.pop_back();
            }
            lines.push_back(line_buffer_);
            line_buffer_.clear();
          } else {
            line_buffer_.push_back(c);
            if (line_buffer_.size() > 512) {
              line_buffer_.clear();
            }
          }
        }
        continue;
      }
      if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
        break;
      }
      if (count == 0) {
        break;
      }
      throw std::runtime_error(std::string("serial read failed: ") + std::strerror(errno));
    }

    return lines;
  }

private:
  int fd_;
  std::string line_buffer_;
};

class Stm32SerialBridge : public rclcpp::Node
{
public:
  Stm32SerialBridge()
  : Node("stm32_serial_bridge"),
    last_cmd_time_(now())
  {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    baudrate_ = declare_parameter<int>("baudrate", 115200);
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    wheel_separation_m_ = declare_parameter<double>("wheel_separation_m", 0.458);
    wheel_radius_m_ = declare_parameter<double>("wheel_radius_m", 0.06);
    max_wheel_rad_s_ = declare_parameter<double>("max_wheel_rad_s", 20.0);
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.5);
    drive_send_rate_hz_ = declare_parameter<double>("drive_send_rate_hz", 20.0);
    tof_fov_degrees_ = declare_parameter<double>("tof_fov_degrees", 65.0);
    tof_range_unit_ = declare_parameter<std::string>("tof_range_unit", "mm");
    tof_min_range_m_ = declare_parameter<double>("tof_min_range_m", 0.05);
    tof_max_range_m_ = declare_parameter<double>("tof_max_range_m", 2.0);
    serial_poll_rate_hz_ = declare_parameter<double>("serial_poll_rate_hz", 100.0);
    log_tx_ = declare_parameter<bool>("log_tx", false);
    log_rx_ = declare_parameter<bool>("log_rx", false);
    open_serial_ = declare_parameter<bool>("open_serial", true);

    validate_parameters();
    make_tof_publishers();
    precompute_tof_angles();

    if (open_serial_) {
      serial_.open(serial_port_, baudrate_);
      RCLCPP_INFO(
        get_logger(), "Opened full-duplex STM32 serial %s at %d baud",
        serial_port_.c_str(), baudrate_);
    } else {
      RCLCPP_WARN(get_logger(), "Serial disabled by open_serial=false");
    }

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        latest_cmd_ = *msg;
        last_cmd_time_ = now();
      });

    const auto drive_period = std::chrono::duration<double>(1.0 / drive_send_rate_hz_);
    drive_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(drive_period),
      std::bind(&Stm32SerialBridge::send_drive_command, this));

    const auto poll_period = std::chrono::duration<double>(1.0 / serial_poll_rate_hz_);
    serial_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(poll_period),
      std::bind(&Stm32SerialBridge::poll_serial, this));
  }

private:
  struct TofFrame
  {
    std::string sensor;
    std::array<double, kTofPointCount> ranges_m;
  };

  void validate_parameters() const
  {
    if (wheel_separation_m_ <= 0.0 || wheel_radius_m_ <= 0.0) {
      throw std::runtime_error("wheel geometry parameters must be positive");
    }
    if (max_wheel_rad_s_ <= 0.0 || command_timeout_s_ <= 0.0) {
      throw std::runtime_error("drive limit and timeout parameters must be positive");
    }
    if (drive_send_rate_hz_ <= 0.0 || serial_poll_rate_hz_ <= 0.0) {
      throw std::runtime_error("timer rates must be positive");
    }
    if (tof_fov_degrees_ <= 0.0 || tof_fov_degrees_ >= 180.0) {
      throw std::runtime_error("tof_fov_degrees must be between 0 and 180");
    }
    if (tof_range_unit_ != "mm" && tof_range_unit_ != "m") {
      throw std::runtime_error("tof_range_unit must be 'mm' or 'm'");
    }
    if (tof_min_range_m_ < 0.0 || tof_max_range_m_ <= tof_min_range_m_) {
      throw std::runtime_error("ToF range limits are invalid");
    }
  }

  void make_tof_publishers()
  {
    tof_publishers_["front_left"] = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tof_cloud/front_left", rclcpp::SensorDataQoS());
    tof_publishers_["front_center"] = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tof_cloud/front_center", rclcpp::SensorDataQoS());
    tof_publishers_["front_right"] = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tof_cloud/front_right", rclcpp::SensorDataQoS());

    tof_frame_ids_["front_left"] = "front_left_tof";
    tof_frame_ids_["front_center"] = "front_center_tof";
    tof_frame_ids_["front_right"] = "front_right_tof";
  }

  void precompute_tof_angles()
  {
    const double fov_rad = deg_to_rad(tof_fov_degrees_);
    const double step = fov_rad / static_cast<double>(kTofGridSize);
    const double first = -0.5 * fov_rad + 0.5 * step;
    for (int i = 0; i < kTofGridSize; ++i) {
      tof_pixel_angles_[i] = first + static_cast<double>(i) * step;
    }
  }

  void send_drive_command()
  {
    geometry_msgs::msg::Twist cmd = latest_cmd_;
    if ((now() - last_cmd_time_).seconds() > command_timeout_s_) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    }

    const double half_track = wheel_separation_m_ * 0.5;
    double left_rad_s = (cmd.linear.x - cmd.angular.z * half_track) / wheel_radius_m_;
    double right_rad_s = (cmd.linear.x + cmd.angular.z * half_track) / wheel_radius_m_;
    left_rad_s = std::clamp(left_rad_s, -max_wheel_rad_s_, max_wheel_rad_s_);
    right_rad_s = std::clamp(right_rad_s, -max_wheel_rad_s_, max_wheel_rad_s_);

    char buffer[80];
    const int size = std::snprintf(
      buffer, sizeof(buffer), "V %.3f %.3f\n", left_rad_s, right_rad_s);
    if (size < 0) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Failed to format drive command");
      return;
    }

    const std::string line(buffer, static_cast<std::size_t>(size));
    if (log_tx_) {
      RCLCPP_INFO(get_logger(), "TX: %s", line.c_str());
    }
    if (!open_serial_) {
      return;
    }

    try {
      serial_.write_all(line);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Drive serial write failed: %s", ex.what());
    }
  }

  void poll_serial()
  {
    if (!open_serial_) {
      return;
    }

    try {
      for (const auto & line : serial_.read_available_lines()) {
        if (log_rx_) {
          RCLCPP_INFO(get_logger(), "RX: %s", line.c_str());
        }
        const auto frame = parse_tof_line(line);
        if (!frame.sensor.empty()) {
          publish_tof_cloud(frame);
        }
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "ToF serial read failed: %s", ex.what());
    }
  }

  TofFrame parse_tof_line(const std::string & line)
  {
    std::istringstream stream(line);
    std::string tag;
    std::string sensor;
    stream >> tag >> sensor;

    if (tag != "TOF") {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring unknown STM32 line: %s", line.c_str());
      return {};
    }

    sensor = normalize_sensor_name(sensor);
    if (tof_publishers_.find(sensor) == tof_publishers_.end()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Unknown ToF sensor name: %s", sensor.c_str());
      return {};
    }

    TofFrame frame;
    frame.sensor = sensor;
    for (int i = 0; i < kTofPointCount; ++i) {
      double raw = 0.0;
      if (!(stream >> raw)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "TOF line must contain 16 values: %s", line.c_str());
        return {};
      }
      frame.ranges_m[static_cast<std::size_t>(i)] =
        tof_range_unit_ == "mm" ? raw * 0.001 : raw;
    }
    return frame;
  }

  void publish_tof_cloud(const TofFrame & frame)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = tof_frame_ids_.at(frame.sensor);
    cloud.height = 1;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(kTofPointCount);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (int row = 0; row < kTofGridSize; ++row) {
      for (int col = 0; col < kTofGridSize; ++col) {
        const double range = frame.ranges_m[static_cast<std::size_t>(row * kTofGridSize + col)];
        if (std::isfinite(range) && range >= tof_min_range_m_ && range <= tof_max_range_m_) {
          const double yaw = tof_pixel_angles_[col];
          const double pitch = -tof_pixel_angles_[row];
          *iter_x = static_cast<float>(range * std::cos(pitch) * std::cos(yaw));
          *iter_y = static_cast<float>(range * std::cos(pitch) * std::sin(yaw));
          *iter_z = static_cast<float>(range * std::sin(pitch));
        } else {
          const auto nan = std::numeric_limits<float>::quiet_NaN();
          *iter_x = nan;
          *iter_y = nan;
          *iter_z = nan;
        }
        ++iter_x;
        ++iter_y;
        ++iter_z;
      }
    }

    cloud.is_dense = false;
    tof_publishers_.at(frame.sensor)->publish(cloud);
  }

  FullDuplexSerial serial_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr drive_timer_;
  rclcpp::TimerBase::SharedPtr serial_timer_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>
    tof_publishers_;
  std::map<std::string, std::string> tof_frame_ids_;

  std::string serial_port_;
  int baudrate_;
  std::string cmd_vel_topic_;
  double wheel_separation_m_;
  double wheel_radius_m_;
  double max_wheel_rad_s_;
  double command_timeout_s_;
  double drive_send_rate_hz_;
  double tof_fov_degrees_;
  std::string tof_range_unit_;
  double tof_min_range_m_;
  double tof_max_range_m_;
  double serial_poll_rate_hz_;
  bool log_tx_;
  bool log_rx_;
  bool open_serial_;

  geometry_msgs::msg::Twist latest_cmd_;
  rclcpp::Time last_cmd_time_;
  std::array<double, kTofGridSize> tof_pixel_angles_ {};
};

}  // namespace care_hw_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<care_hw_bridge::Stm32SerialBridge>());
  rclcpp::shutdown();
  return 0;
}
