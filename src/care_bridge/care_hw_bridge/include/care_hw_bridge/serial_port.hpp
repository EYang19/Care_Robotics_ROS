#ifndef CARE_HW_BRIDGE__SERIAL_PORT_HPP_
#define CARE_HW_BRIDGE__SERIAL_PORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <termios.h>

namespace care_hw_bridge
{

class SerialPort
{
public:
  SerialPort();
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  void open(const std::string & device, int baudrate);
  void close();
  bool is_open() const;
  void write_all(const std::vector<uint8_t> & data);
  void write_all(const std::string & data);

private:
  static speed_t baud_to_termios(int baudrate);

  int fd_;
};

}  // namespace care_hw_bridge

#endif  // CARE_HW_BRIDGE__SERIAL_PORT_HPP_
