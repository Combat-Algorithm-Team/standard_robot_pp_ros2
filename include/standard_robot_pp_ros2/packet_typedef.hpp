// Copyright 2025 SMBU-PolarBear-Robotics-Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef STANDARD_ROBOT_PP_ROS2__PACKET_TYPEDEF_HPP_
#define STANDARD_ROBOT_PP_ROS2__PACKET_TYPEDEF_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace standard_robot_pp_ros2
{
const uint8_t SOF_REFREE_HEAD = 0xA5;
const uint8_t SOF_VISION_HEAD = 0x5A;
const uint8_t SOF_SEND = 0x5A;
const uint8_t SOF_TAIL = 0x55;

// Receive
const uint16_t ID_GAME_STATUS = 0x0001;
const uint16_t ID_GAME_RESULT = 0x0002;
const uint16_t ID_GAME_ROBOT_HP = 0x0003;
const uint16_t ID_EVENT_DATA = 0x0101;
const uint16_t ID_REFREE_WARNNING = 0x0104;
const uint16_t ID_DART_INFO = 0x0105;
const uint16_t ID_ROBOT_STATUS = 0x0201;
const uint16_t ID_POWER_HEAT_DATA = 0x0202;
const uint16_t ID_ROBOT_POS = 0x0203;
const uint16_t ID_BUFF = 0x0204;
const uint16_t ID_HURT_DATA = 0x0206;
const uint16_t ID_SHOOT_DATA = 0x0207;
const uint16_t ID_PROJECTILE_ALLOWANCE = 0x0208;
const uint16_t ID_RFID_STATUS = 0x0209;
const uint16_t ID_DART_CLIENT_CMD = 0x020A;
const uint16_t ID_GROUND_ROBOT_POSITION = 0x020B;
const uint16_t ID_LIDAR_MARK_DATA = 0x020C;
const uint16_t ID_SENTRY_INFO = 0x020D;
const uint16_t ID_RADAR_INFO = 0x020E;

const uint16_t ID_DEBUG = 0x0401;
const uint16_t ID_VISION_DATA = 0x0402;
const uint16_t ID_PID_DEBUG = 0x0403;

// Send
const uint16_t ID_VISION_CMD = 0x01;
const uint16_t ID_NAV_CMD = 0x02;

const uint8_t DEBUG_PACKAGE_NUM = 10;
const uint8_t DEBUG_PACKAGE_NAME_LEN = 10;

struct HeaderFrame
{
  uint8_t sof;   // 数据帧起始字节，固定值为 0xA5
  uint16_t len;  // 数据段长度
  uint8_t seq;   // 数据包序号
  uint8_t crc8;  // 数据帧头的 CRC8 校验
} __attribute__((packed));

/********************************************************/
/* Receive data                                         */
/********************************************************/

// 串口调试数据包
struct DebugPackage
{
  HeaderFrame frame_header;
  uint16_t cmd_id;
  uint32_t time_stamp;

  struct data
  {
    uint8_t name[DEBUG_PACKAGE_NAME_LEN];
    uint8_t type;
    float data;
  } __attribute__((packed)) packages[DEBUG_PACKAGE_NUM];

  uint16_t crc16;
} __attribute__((packed));

// 比赛状态数据包
struct GameStatusPackage
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct data
  {
    uint8_t game_progress;
    uint16_t stage_remain_time;
  } __attribute__((packed));

  uint16_t crc16;
} __attribute__((packed));

// 场地事件数据包
struct EventDataPackage
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct data
  {
    uint32_t ally_supply_zone_non_exchange : 1;
    uint32_t ally_supply_zone_exchange : 1;
    uint32_t ally_supply_zone : 1;

    uint32_t ally_small_power_rune : 2;
    uint32_t ally_big_power_rune : 2;

    uint32_t central_highland : 2;
    uint32_t trapezoidal_highland : 2;

    uint32_t reserved1 : 12;

    uint32_t center_gain_point : 2;
    uint32_t ally_fortress_gain_point : 2;
    uint32_t ally_outpost_gain_point : 2;
    uint32_t base_gain_point : 1;

    uint32_t reserved2 : 2;
  } __attribute__((packed));

  uint16_t crc16;
} __attribute__((packed));

// 机器人性能体系状态数据包
struct RobotStatusPackage
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct data
  {
    uint16_t current_hp;
    uint16_t maximum_hp;
  } __attribute__((packed));

  uint16_t crc16;
} __attribute__((packed));

// 伤害状态数据包
struct HurtDataPackage
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct data
  {
    uint8_t armor_id;
    uint8_t hp_deduction_reason;
  } __attribute__((packed));

  uint16_t crc16;
} __attribute__((packed));

// RFID 状态数据包
struct RfidStatusPackage
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct data
  {
    uint32_t ally_base_gain_point : 1;
    uint32_t ally_central_highland_gain_point : 1;
    uint32_t enemy_central_highland_gain_point : 1;
    uint32_t reserved1 : 14;
    uint32_t ally_fortress_gain_point : 1;
    uint32_t ally_outpost_gain_point : 1;
    uint32_t ally_supply_point_non_exchange : 1;
    uint32_t ally_supply_point_exchange : 1;
    uint32_t reserved2 : 2;
    uint32_t center_gain_point : 1;
    uint32_t enemy_fortress_gain_point : 1;
    uint32_t enemy_outpost_gain_point : 1;
    uint32_t reserved3 : 6;
  } __attribute__((packed));

  uint8_t reserve4;
  uint16_t crc16;
} __attribute__((packed));

// Vision 数据包
struct VisionDataPackage
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct data
  {
    float time_stamp;
    uint8_t enemy_color;
    float pitch;
    float yaw;
    float yaw_diff;
    float bullet_speed;
  } __attribute__((packed));

  uint16_t check_sum;
} __attribute__((packed));

struct ReceiveDataPackage
{
  HeaderFrame frame_header;
  uint16_t cmd_id;

  struct data
  {
    VisionDataPackage::data vision_data;
    GameStatusPackage::data game_status_data;
    EventDataPackage::data event_data;
    RobotStatusPackage::data robot_status_data;
    HurtDataPackage::data hurt_data;
    RfidStatusPackage::data rfid_data;

  } __attribute__((packed)) data;

  uint16_t check_sum;
} __attribute__((packed));

int a = sizeof(ReceiveDataPackage);

/********************************************************/
/* Send data                                            */
/********************************************************/

struct NavigationCmd
{
  HeaderFrame frame_header;

  // uint32_t time_stamp;

  struct
  {
    struct
    {
      float vx;
      float vy;
      float wz;
    } __attribute__((packed)) chassis_vector;

    struct
    {
      uint8_t chassis_spinning;
      uint8_t is_navigating;
    } __attribute__((packed)) control_mode;

  } __attribute__((packed)) data;

} __attribute__((packed));

struct VisionCmd
{
  HeaderFrame frame_header;

  // uint32_t time_stamp;

  struct
  {
    struct
    {
      float pitch;
      float yaw;
      float distance;
    } __attribute__((packed)) gimbal_vector;

    struct
    {
      uint32_t sec;
      uint32_t nanosec;
    } __attribute__((packed)) time_stamp;

    struct
    {
      uint8_t fire_advice;
      uint8_t reserve2;
      uint8_t reserve3;
      uint8_t reserve4;
    } __attribute__((packed)) control_mode;

  } __attribute__((packed)) data;

} __attribute__((packed));

struct SendTestData
{
  uint8_t frame_header;

  struct
  {
    uint8_t fire_advice;  // 0:不开火 1:开火
    uint8_t major_number;
    uint8_t chassis_status;
    float pitch;
    float yaw;
    uint32_t sec;
    uint32_t nanosec;
    float vx;
    float vy;
    float detect_x1;
    float detect_y1;
    float detect_z1;
    uint8_t detect_number1;
    float detect_x2;
    float detect_y2;
    float detect_z2;
    uint8_t detect_number2;
    uint8_t reserve[7];
  } __attribute__((packed)) data;

  uint16_t check_sum;
  uint8_t frame_tail;
} __attribute__((packed));

/********************************************************/
/* template                                             */
/********************************************************/

template <typename T>
inline T fromVector(const std::vector<uint8_t> & data)
{
  T packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

template <typename T>
inline std::vector<uint8_t> toVector(const T & data)
{
  std::vector<uint8_t> packet(sizeof(T));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data), reinterpret_cast<const uint8_t *>(&data) + sizeof(T),
    packet.begin());
  return packet;
}

}  // namespace standard_robot_pp_ros2

#endif  // STANDARD_ROBOT_PP_ROS2__PACKET_TYPEDEF_HPP_
