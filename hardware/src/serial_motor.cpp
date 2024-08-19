#include "artebot_control/serial_motor.hpp"
#include "artebot_control/serial_port.h"
#include "remote.h"
#include "serial_port.h"
#include "crc8.h"

using namespace artebot_control;

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

#define MAX_BUF_SZ 256
// #define DEBUG_SP 1

SerialMotor::SerialMotor(const std::string device, int speed) : device_(device), speed_(speed), fd_(-1)
{
}

SerialMotor::~SerialMotor()
{
    disconnect();
}

void SerialMotor::connect()
{
    fd_ = open_serial_port(device_.c_str(), speed_);
    if (fd_ < 0)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("ArteBotSystemHardware"),
            "Cannot open serial port '%s'.",
            device_.c_str());
        std::__throw_runtime_error("Failed to open serial interface!");
    }
}

void SerialMotor::disconnect()
{
    if (fd_ > 0)
    {
        close_serial_port(fd_);
        fd_ = -1;
    }
}

ssize_t SerialMotor::sendPacket(uint8_t cmd, uint32_t data, uint8_t * packet, size_t packet_sz, size_t recsz)
{
  size_t sz = 0;
  memset(packet, 0, packet_sz);
  packet[sz++] = PACK_BEGIN;
  packet[sz++] = 7;     // len of packet
  packet[sz++] = cmd;   // command to be sent
  memcpy(packet + sz, &data, sizeof(data));
  sz += sizeof(data);
  uint8_t crc = crc8(packet + 2, sz - 2);
  memcpy(packet + sz, &crc, sizeof(crc));
  sz += sizeof(crc);
  packet[sz++] = PACK_END;

  assert(fd_ > 0);
  if (!write_port(fd_, packet, sz))
  {
      if (recsz > 0)
      {
          // receiving back if there is anything to be received
          return read_port(fd_, packet, recsz);
      }
      return 0;
  }
  return -1;
}

struct MotorsOdometry SerialMotor::readOdometry()
{
    size_t sz = 0;
    uint8_t packet[MAX_BUF_SZ];
    struct MotorsOdometry out;
    ssize_t ret = 0;
    do
    {
        ret = sendPacket(CMD_TYPE_CUST,
                         CUSTOM_VEL,
                         packet,
                         MAX_BUF_SZ,
                         sizeof(out));
    } while (ret <= 0);

    // first wheel
    memcpy(&out.vel1, packet + sz, sizeof(out.vel1));
    sz += sizeof(out.vel1);

    // second wheel
    memcpy(&out.vel2, packet + sz, sizeof(out.vel2));
    sz += sizeof(out.vel2);
    return out;
}

void SerialMotor::setMotorsSpeed(double velocity1, double velocity2)
{
    uint8_t packet[MAX_BUF_SZ];
    uint32_t data;
#ifdef DEBUG_SP
    RCLCPP_INFO(rclcpp::get_logger("ArteBotSystemHardware"),
                ">> SP %f %f", velocity1, velocity2);
#endif
    int32_t sp1i = velocity1; // * powf(10, 3);
    int32_t sp2i = velocity2; // * powf(10, 3);
    uint32_t to_copy = (sp1i << 16) | (sp2i & 0xffff);
    memcpy((uint8_t *)&data, (uint8_t *)&to_copy, sizeof(data));

    sendPacket(CMD_TYPE_SP, data, packet, MAX_BUF_SZ, 0);
}
