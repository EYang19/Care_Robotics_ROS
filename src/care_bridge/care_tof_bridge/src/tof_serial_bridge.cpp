#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace care_tof_bridge
{
namespace
{

constexpr int kGridSize = 4;
constexpr int kPointCount = kGridSize * kGridSize;

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

class SerialLineReader
{
public:
  SerialLineReader()
  : fd_(-1)
  {
  }

  ~SerialLineReader()
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

    const auto speed = baud_to_termios(baudrate);
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

class TofSerialBridge : public rclcpp::Node
{
public:
  TofSerialBridge()
  : Node("tof_serial_bridge")
  {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    baudrate_ = declare_parameter<int>("baudrate", 115200);
    fov_degrees_ = declare_parameter<double>("fov_degrees", 65.0);
    range_unit_ = declare_parameter<std::string>("range_unit", "mm");
    min_range_m_ = declare_parameter<double>("min_range_m", 0.05);
    max_range_m_ = declare_parameter<double>("max_range_m", 2.0);
    poll_rate_hz_ = declare_parameter<double>("poll_rate_hz", 100.0);
    open_serial_ = declare_parameter<bool>("open_serial", true);
    log_lines_ = declare_parameter<bool>("log_lines", false);

    validate_parameters();
    make_publishers();
    precompute_pixel_angles();

    if (open_serial_) {
      serial_.open(serial_port_, baudrate_);
      RCLCPP_INFO(
        get_logger(), "Opened %s at %d baud for ToF ASCII input",
        serial_port_.c_str(), baudrate_);
    } else {
      RCLCPP_WARN(get_logger(), "Serial input disabled by open_serial=false");
    }

    const auto period = std::chrono::duration<double>(1.0 / poll_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TofSerialBridge::poll_serial, this));
  }

private:
  struct ParsedFrame
  {
    std::string sensor;
    std::array<double, kPointCount> ranges_m;
  };

  void validate_parameters() const
  {
    if (fov_degrees_ <= 0.0 || fov_degrees_ >= 180.0) {
      throw std::runtime_error("fov_degrees must be between 0 and 180");
    }
    if (range_unit_ != "mm" && range_unit_ != "m") {
      throw std::runtime_error("range_unit must be 'mm' or 'm'");
    }
    if (min_range_m_ < 0.0 || max_range_m_ <= min_range_m_) {
      throw std::runtime_error("range limits are invalid");
    }
    if (poll_rate_hz_ <= 0.0) {
      throw std::runtime_error("poll_rate_hz must be positive");
    }
  }

  void make_publishers()
  {
    publishers_["front_left"] = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tof_cloud/front_left", rclcpp::SensorDataQoS());
    publishers_["front_center"] = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tof_cloud/front_center", rclcpp::SensorDataQoS());
    publishers_["front_right"] = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/tof_cloud/front_right", rclcpp::SensorDataQoS());

    frame_ids_["front_left"] = "front_left_tof";
    frame_ids_["front_center"] = "front_center_tof";
    frame_ids_["front_right"] = "front_right_tof";
  }

  void precompute_pixel_angles()
  {
    const double fov_rad = deg_to_rad(fov_degrees_);
    const double step = fov_rad / static_cast<double>(kGridSize);
    const double first = -0.5 * fov_rad + 0.5 * step;

    for (int i = 0; i < kGridSize; ++i) {
      pixel_angles_[i] = first + static_cast<double>(i) * step;
    }
  }

  void poll_serial()
  {
    if (!open_serial_) {
      return;
    }

    try {
      for (const auto & line : serial_.read_available_lines()) {
        if (log_lines_) {
          RCLCPP_INFO(get_logger(), "RX: %s", line.c_str());
        }
        const auto parsed = parse_line(line);
        if (parsed.sensor.empty()) {
          continue;
        }
        publish_cloud(parsed);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "ToF serial read failed: %s", ex.what());
    }
  }

  ParsedFrame parse_line(const std::string & line)
  {
    std::istringstream stream(line);
    std::string tag;
    std::string sensor;
    stream >> tag >> sensor;

    if (tag != "TOF") {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring non-TOF serial line: %s", line.c_str());
      return {};
    }

    sensor = normalize_sensor_name(sensor);
    if (publishers_.find(sensor) == publishers_.end()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Unknown ToF sensor name: %s", sensor.c_str());
      return {};
    }

    ParsedFrame frame;
    frame.sensor = sensor;

    for (int i = 0; i < kPointCount; ++i) {
      double raw = 0.0;
      if (!(stream >> raw)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TOF line must contain 16 range values: %s", line.c_str());
        return {};
      }
      frame.ranges_m[static_cast<std::size_t>(i)] = convert_to_meters(raw);
    }

    return frame;
  }

  double convert_to_meters(double raw) const
  {
    if (range_unit_ == "mm") {
      return raw * 0.001;
    }
    return raw;
  }

  void publish_cloud(const ParsedFrame & frame)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = frame_ids_.at(frame.sensor);
    cloud.height = 1;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(kPointCount);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (int row = 0; row < kGridSize; ++row) {
      for (int col = 0; col < kGridSize; ++col) {
        const auto index = static_cast<std::size_t>(row * kGridSize + col);
        const double range = frame.ranges_m[index];

        if (std::isfinite(range) && range >= min_range_m_ && range <= max_range_m_) {
          const double yaw = pixel_angles_[col];
          const double pitch = -pixel_angles_[row];
          const double x = range * std::cos(pitch) * std::cos(yaw);
          const double y = range * std::cos(pitch) * std::sin(yaw);
          const double z = range * std::sin(pitch);

          *iter_x = static_cast<float>(x);
          *iter_y = static_cast<float>(y);
          *iter_z = static_cast<float>(z);
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
    publishers_.at(frame.sensor)->publish(cloud);
  }

  SerialLineReader serial_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> publishers_;
  std::map<std::string, std::string> frame_ids_;

  std::string serial_port_;
  int baudrate_;
  double fov_degrees_;
  std::string range_unit_;
  double min_range_m_;
  double max_range_m_;
  double poll_rate_hz_;
  bool open_serial_;
  bool log_lines_;
  std::array<double, kGridSize> pixel_angles_ {};
};

}  // namespace care_tof_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<care_tof_bridge::TofSerialBridge>());
  rclcpp::shutdown();
  return 0;
}
