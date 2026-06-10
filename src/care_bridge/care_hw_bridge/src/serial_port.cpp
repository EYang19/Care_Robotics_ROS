#include "care_hw_bridge/serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace care_hw_bridge
{

SerialPort::SerialPort()
: fd_(-1)
{
}

SerialPort::~SerialPort()
{
  close();
}

void SerialPort::open(const std::string & device, int baudrate)
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

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::is_open() const
{
  return fd_ >= 0;
}

void SerialPort::write_all(const std::vector<uint8_t> & data)
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

void SerialPort::write_all(const std::string & data)
{
  write_all(std::vector<uint8_t>(data.begin(), data.end()));
}

speed_t SerialPort::baud_to_termios(int baudrate)
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

}  // namespace care_hw_bridge
