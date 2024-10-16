#ifndef __MA_SERIAL_H__
#define __MA_SERIAL_H__

#include <serial/serial.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

namespace ma_serial_packet
{

  /**
   * SendPacket
   *
   */
  struct SendPacket
  {
    uint8_t head = 0x5A;
    uint16_t data_lenth;
    uint8_t seq;
    u_int8_t cpc;
    // ID
    uint16_t id;
    // 数据
    uint16_t target_rabot_id;
    float linear_x;
    float linear_y;
    uint16_t checksum = 0;
  } __attribute__((packed));

  /***
   * @details: Receive odometry msg.
   * @param: head, v_x, v_y, v_z, p_x, p_y, p_z, checksum
   */
  struct ReceivePacket
  {
    // 头部
    uint8_t head = 0xA5;
    uint16_t data_lenth;
    uint8_t seq;
    u_int8_t cpc;
    // ID
    uint16_t id;
    // 进度
    uint8_t mark_hero_progress;
    uint8_t mark_engineer_progress;
    uint8_t mark_standard_3_progress;
    uint8_t mark_standard_4_progress;
    uint8_t mark_standard_5_progress;
    uint8_t mark_sentry_progress;
    uint16_t checksum = 0;
  } __attribute__((packed));

  struct ReceiveMianPacket
  {
    // 头部
    uint8_t head = 0x5A;
    uint16_t data_lenth;
    uint8_t seq;
    u_int8_t cpc;
    // ID
    uint16_t id;
    // 是否双倍免伤
    uint8_t radar_info;
    uint16_t checksum = 0;
  } __attribute__((packed));
  /**
   * Alter vector above into struct and utilize it.
   */
  inline ReceivePacket fromVector(const std::vector<uint8_t> &data)
  {
    ReceivePacket packet;
    std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
    return packet;
  }
  inline ReceiveMianPacket fromVector1(const std::vector<uint8_t> &data)
  {
    ReceiveMianPacket packet;
    std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
    return packet;
  }
  /**
   * Alter struct object above into vector and utilize it.
   */
  inline std::vector<uint8_t> toVector(const SendPacket &data)
  {
    std::vector<uint8_t> packet(sizeof(SendPacket));
    std::copy(
        reinterpret_cast<const uint8_t *>(&data),
        reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
    return packet;
  }

}

#endif