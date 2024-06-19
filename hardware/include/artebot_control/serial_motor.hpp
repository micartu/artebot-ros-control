#ifndef ARTEBOT_CONTROL_SERIAL_MOTOR_HPP_
#define ARTEBOT_CONTROL_SERIAL_MOTOR_HPP_

#include <cstdint>
#include <string>
#include <sys/types.h>

namespace artebot_control
{

  struct MotorsOdometry
  {
    float vel1, vel2;
    uint32_t pos1, pos2;
  };

  class SerialMotor
  {
  public:
    SerialMotor(const std::string device, int speed);
    ~SerialMotor();

    void connect();
    void disconnect();
    struct MotorsOdometry readOdometry();
    void setMotorsSpeed(double velocity1, double velocity2);

  private:
    /// @brief device name
    const std::string  device_;
    /// @brief handle to serial which controls the motors
    int speed_;
    /// @brief handle to serial which controls the motors
    int fd_;
    /// sends a packet into the opened port
    ssize_t sendPacket(uint8_t cmd, uint32_t data, uint8_t *packet, size_t packet_sz, size_t minsz);
  };
} // namespace artebot_control

#endif // ARTEBOT_CONTROL_SERIAL_MOTOR_HPP_