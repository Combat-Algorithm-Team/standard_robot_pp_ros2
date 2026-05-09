// Copyright 2026 NJUST-Combat-Robotics-Team
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

#include "standard_robot_pp_ros2/standard_robot_pp_ros2.hpp"

#include <algorithm>
#include <bitset>
#include <chrono>
#include <cstdint>
#include <memory>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <thread>

#include "standard_robot_pp_ros2/crc8_crc16.hpp"
#include "standard_robot_pp_ros2/packet_typedef.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

namespace standard_robot_pp_ros2
{
namespace
{
constexpr auto kUsbNotOkSleepTime = std::chrono::milliseconds(1000);
constexpr auto kUsbProtectSleepTime = std::chrono::milliseconds(1000);
constexpr auto kUsbProtectReconnectTime = std::chrono::milliseconds(1000);
constexpr double kUsbTimeoutSeconds = 0.5;
constexpr double kUsbReconnectGraceSeconds = 2.0;
constexpr int kSerialDeviceSearchCount = 10;
}  // namespace

StandardRobotPpRos2Node::StandardRobotPpRos2Node(const rclcpp::NodeOptions & options)
: Node("StandardRobotPpRos2Node", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start StandardRobotPpRos2Node!");

  getParams();
  createPublisher();
  createSubscription();
  last_receive_time_ = this->now();
  last_reconnect_time_ = last_receive_time_;
  pkg_last_receive_time_ = 0.0f;
  is_usb_ok_ = false;

  serial_port_protect_thread_ = std::thread(&StandardRobotPpRos2Node::serialPortProtect, this);
  receive_thread_ = std::thread(&StandardRobotPpRos2Node::receiveData, this);
  send_thread_ = std::thread(&StandardRobotPpRos2Node::sendData, this);
}

StandardRobotPpRos2Node::~StandardRobotPpRos2Node()
{
  if (send_thread_.joinable()) {
    send_thread_.join();
  }

  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_port_protect_thread_.joinable()) {
    serial_port_protect_thread_.join();
  }

  closeSerialPort("node shutdown");

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void StandardRobotPpRos2Node::createPublisher()
{
  game_status_pub_ =
    this->create_publisher<combat_rm_interfaces::msg::GameStatus>("referee/game_status", 10);
  game_robot_hp_pub_ =
    this->create_publisher<combat_rm_interfaces::msg::GameRobotHp>("referee/game_robot_hp", 10);
  event_data_pub_ =
    this->create_publisher<combat_rm_interfaces::msg::EventData>("referee/event_data", 10);
  robot_status_pub_ =
    this->create_publisher<combat_rm_interfaces::msg::RobotStatus>("referee/robot_status", 10);
  robot_pos_pub_ =
    this->create_publisher<combat_rm_interfaces::msg::RobotPos>("referee/robot_pos", 10);
  hurt_data_pub_ =
    this->create_publisher<combat_rm_interfaces::msg::HurtData>("referee/hurt_data", 10);
  rfid_status_pub_ =
    this->create_publisher<combat_rm_interfaces::msg::RfidStatus>("referee/rfid_status", 10);
  robot_status_pub_ =
    this->create_publisher<combat_rm_interfaces::msg::RobotStatus>("referee/robot_status", 10);
  ground_robot_position_pub_ =
    this->create_publisher<combat_rm_interfaces::msg::GroundRobotPosition>(
      "referee/ground_robot_position", 10);
  joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>("gimbal_joint_publisher", 10);
}

void StandardRobotPpRos2Node::createSubscription()
{
  cmd_chassis_status_sub_ = this->create_subscription<example_interfaces::msg::UInt8>(
    "cmd_chassis_status", 1,
    std::bind(&StandardRobotPpRos2Node::cmdChassisStatusCallback, this, std::placeholders::_1));

  cmd_sentry_status_sub_ = this->create_subscription<example_interfaces::msg::UInt8>(
    "cmd_sentry_status", 1,
    std::bind(&StandardRobotPpRos2Node::cmdSentryStatusCallback, this, std::placeholders::_1));

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&StandardRobotPpRos2Node::cmdVelCallback, this, std::placeholders::_1));
}

void StandardRobotPpRos2Node::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    nav_k_ = declare_parameter<double>("nav_k", 1.0);
    nav_k_ = this->get_parameter("nav_k").as_double();
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The nav k provided was invalid");
    throw ex;
  }

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);

  record_rosbag_ = declare_parameter("record_rosbag", false);
  debug_ = declare_parameter("debug", false);
}

bool StandardRobotPpRos2Node::isSerialPortOpen()
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  return serial_driver_ && serial_driver_->port() && serial_driver_->port()->is_open();
}

bool StandardRobotPpRos2Node::shouldReconnectSerialPort()
{
  if (!is_usb_ok_) {
    return true;
  }

  if (!isSerialPortOpen()) {
    RCLCPP_WARN(get_logger(), "Serial port is not open, try to reconnect...");
    return true;
  }

  const auto now = this->now();
  const double receive_dt = (now - last_receive_time_).seconds();
  const double reconnect_dt = (now - last_reconnect_time_).seconds();

  if (receive_dt > kUsbTimeoutSeconds && reconnect_dt > kUsbReconnectGraceSeconds) {
    RCLCPP_WARN(get_logger(), "No data timeout: %.2f sec -> reconnect", receive_dt);
    return true;
  }

  return false;
}

void StandardRobotPpRos2Node::closeSerialPort(const std::string & failure_context)
{
  try {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_driver_ && serial_driver_->port() && serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Failed to close serial port during %s: %s", failure_context.c_str(),
      ex.what());
  }
}

std::string StandardRobotPpRos2Node::getSerialDeviceSearchBase() const
{
  std::string base_path = device_name_;
  const auto last_non_digit = base_path.find_last_not_of("0123456789");
  if (last_non_digit != std::string::npos && last_non_digit + 1 < base_path.size()) {
    base_path = base_path.substr(0, last_non_digit + 1);
  }
  return base_path;
}

bool StandardRobotPpRos2Node::findSerialDevice()
{
  const std::string base_path = getSerialDeviceSearchBase();

  for (int i = 0; i < kSerialDeviceSearchCount; ++i) {
    const std::string candidate = base_path + std::to_string(i);
    if (rcpputils::fs::exists(candidate)) {
      RCLCPP_INFO(get_logger(), "Serial File %s found!", candidate.c_str());
      device_name_ = candidate;
      return true;
    }

    RCLCPP_WARN(get_logger(), "Serial File %s not found! Trying next...", candidate.c_str());
  }

  RCLCPP_ERROR(get_logger(), "No serial port found in range 0-9 with base %s", base_path.c_str());
  return false;
}

bool StandardRobotPpRos2Node::openSerialPort()
{
  std::lock_guard<std::mutex> lock(serial_mutex_);

  if (serial_driver_) {
    serial_driver_->init_port(device_name_, *device_config_);
  }

  if (serial_driver_->port() && !serial_driver_->port()->is_open()) {
    serial_driver_->port()->open();
  }

  return serial_driver_->port() && serial_driver_->port()->is_open();
}

void StandardRobotPpRos2Node::reconnectSerialPort()
{
  reconnecting_serial_ = true;
  is_usb_ok_ = false;

  closeSerialPort("serial reconnection");
  std::this_thread::sleep_for(kUsbProtectReconnectTime);

  if (!findSerialDevice()) {
    reconnecting_serial_ = false;
    return;
  }

  if (openSerialPort()) {
    RCLCPP_INFO(get_logger(), "Serial port %s opened successfully!", device_name_.c_str());
    last_reconnect_time_ = this->now();
    last_receive_time_ = last_reconnect_time_;
    is_usb_ok_ = true;
  } else {
    RCLCPP_ERROR(
      get_logger(), "Serial port %s open failed (port is null or open failed)",
      device_name_.c_str());
  }

  reconnecting_serial_ = false;
}

/********************************************************/
/* Serial port protect                                  */
/********************************************************/
void StandardRobotPpRos2Node::serialPortProtect()
{
  RCLCPP_INFO(get_logger(), "Start serialPortProtect!");

  while (rclcpp::ok()) {
    try {
      if (shouldReconnectSerialPort()) {
        reconnectSerialPort();
      } else {
        is_usb_ok_ = true;
      }
    }
    // catch exception
    catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Serial port exception: %s", ex.what());
      reconnecting_serial_ = false;
      is_usb_ok_ = false;
      closeSerialPort("serial protect exception");
    }
    // sleep for a while before next check
    std::this_thread::sleep_for(kUsbProtectSleepTime);
  }

  // close serial port on exit
  RCLCPP_INFO(get_logger(), "serialPortProtect exit, close serial port");
  closeSerialPort("serial protect exit");
  is_usb_ok_ = false;
}

/********************************************************/
/* Receive data                                         */
/********************************************************/

std::vector<uint8_t> StandardRobotPpRos2Node::receivePacket()
{
  std::vector<uint8_t> sof(1);
  std::vector<uint8_t> data_buf;

  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (!serial_driver_ || !serial_driver_->port() || !serial_driver_->port()->is_open()) {
    throw std::runtime_error("serial port is not open");
  }

  serial_driver_->port()->receive(sof);
  if (sof[0] != SOF_HEAD) {
    return data_buf;
  }

  data_buf.resize(PACKAGE_LENGTH);
  data_buf[0] = sof[0];

  std::vector<uint8_t> receive_buf(PACKAGE_LENGTH - 1);
  int received_len = serial_driver_->port()->receive(receive_buf);
  if (received_len <= 0) {
    throw std::runtime_error("serial receive returned no payload data");
  }

  std::copy(receive_buf.begin(), receive_buf.begin() + received_len, data_buf.begin() + 1);

  int received_len_sum = received_len;
  int remain_len = PACKAGE_LENGTH - 1 - received_len;
  while (remain_len > 0) {
    std::vector<uint8_t> remain_buf(remain_len);
    received_len = serial_driver_->port()->receive(remain_buf);
    if (received_len <= 0) {
      throw std::runtime_error("serial receive returned no remaining data");
    }

    std::copy(
      remain_buf.begin(), remain_buf.begin() + received_len,
      data_buf.begin() + 1 + received_len_sum);
    received_len_sum += received_len;
    remain_len -= received_len;
  }

  return data_buf;
}

bool StandardRobotPpRos2Node::handleReceivePacket(std::vector<uint8_t> & data_buf)
{
  if (data_buf[PACKAGE_LENGTH - 3] != SOF_TAIL) {
    RCLCPP_ERROR(get_logger(), "Data tail error!");
    return false;
  }

  if (!checksum::verify_check_sum16(data_buf)) {
    RCLCPP_ERROR(get_logger(), "Data segment check sum error!");
    return false;
  }

  const float current_receive_time = fromVector<float>(data_buf, 2);
  const float dt = current_receive_time - pkg_last_receive_time_;
  if (dt <= RECEIVE_TIMEOUT) {
    RCLCPP_WARN(get_logger(), "Receive data timeout! dt: %.2f s", dt);
    return false;
  }
  pkg_last_receive_time_ = current_receive_time;

  switch (data_buf[1]) {
    case RECEIVE_VISION_ID: {
      const GimbalToVision gimbal_to_vision = fromVector<GimbalToVision>(data_buf);

      publish(gimbal_to_vision.yaw_diff, gimbal_to_vision.pitch);
      pkg_last_receive_time_ = gimbal_to_vision.DWT_stamp;
      break;
    }
    case RECEIVE_REFEREE1_ID: {
      const RefereePackage1 referee_package1 = fromVector<RefereePackage1>(data_buf);

      publish(referee_package1.game_status_data);
      publish(referee_package1.event_data);
      publish(referee_package1.robot_status_data);
      publish(referee_package1.hurt_data);
      publish(referee_package1.rfid_status_data);
      pkg_last_receive_time_ = referee_package1.DWT_stamp;
      break;
    }
    case RECEIVE_REFEREE2_ID: {
      const RefereePackage2 referee_package2 = fromVector<RefereePackage2>(data_buf);

      publish(referee_package2.robot_pos_data);
      publish(referee_package2.ground_robot_pos_data);
      publish(referee_package2.game_robot_hp_data);
      pkg_last_receive_time_ = referee_package2.DWT_stamp;
      break;
    }
    default:
      RCLCPP_ERROR(get_logger(), "Unknown data package received! ID: %d", data_buf[1]);
      return false;
  }

  last_receive_time_ = this->now();
  return true;
}

void StandardRobotPpRos2Node::handleSerialIoError(
  const std::string & operation, const std::exception & ex)
{
  RCLCPP_ERROR(get_logger(), "Error %s data: %s", operation.c_str(), ex.what());
  is_usb_ok_ = false;
  closeSerialPort(operation + " error");
}

void StandardRobotPpRos2Node::receiveData()
{
  RCLCPP_INFO(get_logger(), "Start receiveData!");

  int time_waiting = 0;

  while (rclcpp::ok()) {
    // 串口状态
    if (!is_usb_ok_ || reconnecting_serial_) {
      RCLCPP_WARN(get_logger(), "Receive: Usb is not ok! Wait for : %d s", time_waiting++);
      std::this_thread::sleep_for(kUsbNotOkSleepTime);
      continue;
    }

    try {
      auto data_buf = receivePacket();
      if (data_buf.empty()) {
        continue;
      }
      if (handleReceivePacket(data_buf)) {
        time_waiting = 0;
      }
    } catch (const std::exception & ex) {
      handleSerialIoError("receiving", ex);
    }
  }
}

void StandardRobotPpRos2Node::publish(const GameStatusPackage::data & pkg)
{
  combat_rm_interfaces::msg::GameStatus msg;

  msg.game_progress = pkg.game_progress;
  msg.stage_remain_time = pkg.stage_remain_time;

  game_status_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publish(const GameRobotHpPackage::data & pkg)
{
  combat_rm_interfaces::msg::GameRobotHp msg;

  msg.hero_hp = pkg.hero_hp;
  msg.engineer_hp = pkg.engineer_hp;
  msg.standard_3_hp = pkg.standard_3_hp;
  msg.standard_4_hp = pkg.standard_4_hp;
  msg.ally_outpost_hp = pkg.ally_outpost_hp;
  msg.ally_base_hp = pkg.ally_base_hp;

  game_robot_hp_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publish(const EventDataPackage::data & pkg)
{
  combat_rm_interfaces::msg::EventData msg;

  msg.ally_supply_zone_non_exchange = pkg.ally_supply_zone_non_exchange;
  msg.ally_supply_zone_exchange = pkg.ally_supply_zone_exchange;
  msg.ally_supply_zone = pkg.ally_supply_zone;
  msg.ally_small_power_rune = pkg.ally_small_power_rune;
  msg.ally_big_power_rune = pkg.ally_big_power_rune;
  msg.central_highland = pkg.central_highland;
  msg.trapezoidal_highland = pkg.trapezoidal_highland;
  msg.center_gain_point = pkg.center_gain_point;
  msg.ally_fortress_gain_point = pkg.ally_fortress_gain_point;
  msg.ally_outpost_gain_point = pkg.ally_outpost_gain_point;
  msg.base_gain_point = pkg.base_gain_point;

  event_data_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publish(const RobotStatusPackage::data & pkg)
{
  combat_rm_interfaces::msg::RobotStatus msg;

  msg.current_hp = pkg.current_hp;
  msg.maximum_hp = pkg.maximum_hp;

  robot_status_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publish(const RobotPosPackage::data & pkg)
{
  combat_rm_interfaces::msg::RobotPos msg;

  msg.x = pkg.x;
  msg.y = pkg.y;

  robot_pos_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publish(const HurtDataPackage::data & pkg)
{
  combat_rm_interfaces::msg::HurtData msg;

  msg.armor_id = pkg.armor_id;
  msg.hp_deduction_reason = pkg.hp_deduction_reason;

  hurt_data_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publish(const RfidStatusPackage::data & pkg)
{
  combat_rm_interfaces::msg::RfidStatus msg;

  msg.ally_base_gain_point = pkg.ally_base_gain_point;
  msg.ally_central_highland_gain_point = pkg.ally_central_highland_gain_point;
  msg.enemy_central_highland_gain_point = pkg.enemy_central_highland_gain_point;
  msg.ally_fortress_gain_point = pkg.ally_fortress_gain_point;
  msg.ally_outpost_gain_point = pkg.ally_outpost_gain_point;
  msg.ally_supply_point_non_exchange = pkg.ally_supply_point_non_exchange;
  msg.ally_supply_point_exchange = pkg.ally_supply_point_exchange;
  msg.center_gain_point = pkg.center_gain_point;
  msg.enemy_fortress_gain_point = pkg.enemy_fortress_gain_point;
  msg.enemy_outpost_gain_point = pkg.enemy_outpost_gain_point;

  rfid_status_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publish(const GroundRobotPositionPackage::data & pkg)
{
  combat_rm_interfaces::msg::GroundRobotPosition msg;

  msg.hero_position.x = pkg.hero_x;
  msg.hero_position.y = pkg.hero_y;
  msg.engineer_position.x = pkg.engineer_x;
  msg.engineer_position.y = pkg.engineer_y;
  msg.standard_3_position.x = pkg.standard_3_x;
  msg.standard_3_position.y = pkg.standard_3_y;
  msg.standard_4_position.x = pkg.standard_4_x;
  msg.standard_4_position.y = pkg.standard_4_y;

  ground_robot_position_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publish(float yaw_diff, float pitch)
{
  sensor_msgs::msg::JointState joint_msg;
  joint_msg.header.stamp = this->now();
  joint_msg.name = {
    "gimbal_yaw_joint",
    "gimbal_pitch_joint",
  };
  joint_msg.position = {
    static_cast<double>(yaw_diff),
    static_cast<double>(pitch),
  };
  joint_state_pub_->publish(joint_msg);
}

/********************************************************/
/* Send data                                            */
/********************************************************/
void StandardRobotPpRos2Node::initNavToGimbalData()
{
  nav_to_gimbal_data_.head[0] = SOF_HEAD;
  nav_to_gimbal_data_.head[1] = 0x02;
  nav_to_gimbal_data_.tail = SOF_TAIL;
  nav_to_gimbal_data_.check_sum = 0;
  nav_to_gimbal_data_.time_stamp = 0;
  nav_to_gimbal_data_.chassis_status = 0;
  nav_to_gimbal_data_.sentry_status = 0;
  nav_to_gimbal_data_.mode = 0.0;
  nav_to_gimbal_data_.vx = 0.0;
  nav_to_gimbal_data_.vy = 0.0;
  nav_to_gimbal_data_.vyaw = 0.0;
}

void StandardRobotPpRos2Node::sendNavToGimbalData()
{
  checksum::append_check_sum(
    reinterpret_cast<uint8_t *>(&nav_to_gimbal_data_), sizeof(NavToGimbal));
  const std::vector<uint8_t> send_data = toVector(nav_to_gimbal_data_);

  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (!serial_driver_ || !serial_driver_->port() || !serial_driver_->port()->is_open()) {
    throw std::runtime_error("serial port is not open");
  }
  serial_driver_->port()->send(send_data);
}

void StandardRobotPpRos2Node::sendData()
{
  RCLCPP_INFO(get_logger(), "Start send Data!");

  initNavToGimbalData();

  int retry_count = 0;

  while (rclcpp::ok()) {
    if (!is_usb_ok_ || reconnecting_serial_) {
      RCLCPP_WARN(get_logger(), "send: usb is not ok! Retry count: %d", retry_count++);
      std::this_thread::sleep_for(kUsbNotOkSleepTime);
      continue;
    }

    try {
      sendNavToGimbalData();
      retry_count = 0;
    } catch (const std::exception & ex) {
      handleSerialIoError("sending", ex);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

void StandardRobotPpRos2Node::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // nav_k_ = this->get_parameter("nav_k").as_double();
  nav_to_gimbal_data_.time_stamp = static_cast<uint64_t>(this->get_clock()->now().nanoseconds());
  nav_to_gimbal_data_.vx = msg->linear.x * nav_k_;
  nav_to_gimbal_data_.vy = msg->linear.y * nav_k_;
}

void StandardRobotPpRos2Node::cmdChassisStatusCallback(
  const example_interfaces::msg::UInt8::SharedPtr msg)
{
  nav_to_gimbal_data_.chassis_status = msg->data;
}

void StandardRobotPpRos2Node::cmdSentryStatusCallback(
  const example_interfaces::msg::UInt8::SharedPtr msg)
{
  nav_to_gimbal_data_.sentry_status = msg->data;
}

// void StandardRobotPpRos2Node::checkTargetInRegionCallback(const std_msgs::msg::Bool::SharedPtr msg)
// {
//   check_target_in_region_ = msg->data;
// }

bool StandardRobotPpRos2Node::callTriggerService(const std::string & service_name)
{
  auto client = this->create_client<std_srvs::srv::Trigger>(service_name);
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto start_time = std::chrono::steady_clock::now();
  while (!client->wait_for_service(0.1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        get_logger(), "Interrupted while waiting for the service: %s", service_name.c_str());
      return false;
    }
    auto elapsed_time = std::chrono::steady_clock::now() - start_time;
    if (elapsed_time > std::chrono::seconds(5)) {
      RCLCPP_ERROR(
        get_logger(), "Service %s not available after 5 seconds, giving up.", service_name.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Service %s not available, waiting again...", service_name.c_str());
  }

  auto result = client->async_send_request(request);
  if (
    rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(
      get_logger(), "Service %s call succeeded: %s", service_name.c_str(),
      result.get()->success ? "true" : "false");
    return result.get()->success;
  }

  RCLCPP_ERROR(get_logger(), "Service %s call failed", service_name.c_str());
  return false;
}

}  // namespace standard_robot_pp_ros2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(standard_robot_pp_ros2::StandardRobotPpRos2Node)
