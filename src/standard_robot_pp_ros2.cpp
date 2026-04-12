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

#include <memory>
#include <chrono>
#include <thread>
#include <cstdint>
#include <bitset>
#include <Eigen/Geometry>
#include <rclcpp/logging.hpp>

#include "standard_robot_pp_ros2/crc8_crc16.hpp"
#include "standard_robot_pp_ros2/packet_typedef.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <tf2/LinearMath/Matrix3x3.h>


#define USB_NOT_OK_SLEEP_TIME 1000   // (ms)
#define USB_PROTECT_SLEEP_TIME 1000  // (ms)
#define USB_TIMEOUT 0.5              // (s)
#define USB_PROTECT_RECONNECT_TIME 1000

using namespace std::chrono_literals;

namespace standard_robot_pp_ros2
{

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
  pkg_last_receive_time_ = 0.0f;
  is_usb_ok_ = false;
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

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

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

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
    this->create_publisher<combat_rm_interfaces::msg::GroundRobotPosition>("referee/ground_robot_position", 10);
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

/********************************************************/
/* Serial port protect                                  */
/********************************************************/
void StandardRobotPpRos2Node::serialPortProtect()
{
  RCLCPP_INFO(get_logger(), "Start serialPortProtect!");

  while (rclcpp::ok()) {
    try {
      bool serial_error = !is_usb_ok_;
      if (!serial_driver_ || !serial_driver_->port() || !serial_driver_->port()->is_open()) {
        RCLCPP_WARN(get_logger(), "Serial port is not open, try to reconnect...");
        serial_error = true;
      } else if (!serial_error) {
        auto now = this->now();
        double dt = (now - last_receive_time_).seconds();

        if (dt > USB_TIMEOUT) {
          RCLCPP_WARN(get_logger(), "No data timeout: %.2f sec → reconnect", dt);
          serial_error = true;
        }
      }
      if (serial_error) {
        is_usb_ok_ = false;
        try {
          if (serial_driver_ && serial_driver_->port() && serial_driver_->port()->is_open()) {
            serial_driver_->port()->close();
          }
        } catch (const std::exception & ex) {
          RCLCPP_ERROR(get_logger(), "Serial port reconnection failed: %s, close", ex.what());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(USB_PROTECT_RECONNECT_TIME));

        bool found_port = false;
        std::string base_path = device_name_;
        size_t last_non_digit = base_path.find_last_not_of("0123456789");
        if (last_non_digit != std::string::npos && last_non_digit + 1 < base_path.size()) {
          base_path = base_path.substr(0, last_non_digit + 1);  // 去掉末尾数字
        }
        for (int i = 0; i < 10; ++i) {
          std::string candidate = base_path + std::to_string(i);
          if (rcpputils::fs::exists(candidate)) {
            // 找到设备
            RCLCPP_INFO(get_logger(), "Serial File %s found!", candidate.c_str());
            device_name_ = candidate;
            found_port = true;
            break;
          } else {
            RCLCPP_WARN(
              get_logger(), "Serial File %s not found! Trying next...", candidate.c_str());
          }
        }

        if (found_port) {
          // reinit serial driver and port
          if (serial_driver_) {
            serial_driver_->init_port(device_name_, *device_config_);
          }
          // try to open the port
          if (serial_driver_->port() && !serial_driver_->port()->is_open()) {
            serial_driver_->port()->open();
          }
          // check if the port is open
          if (serial_driver_->port()->is_open()) {
            RCLCPP_INFO(get_logger(), "Serial port %s opened successfully!", device_name_.c_str());
            is_usb_ok_ = true;
            last_receive_time_ = this->now();
          } else {
            RCLCPP_ERROR(
              get_logger(), "Serial port %s open failed (port is null or open failed)",
              device_name_.c_str());
            is_usb_ok_ = false;
          }
        } else {
          RCLCPP_ERROR(
            get_logger(), "No serial port found in range 0-9 with base %s", base_path.c_str());
          is_usb_ok_ = false;
        }
      } else {
        is_usb_ok_ = true;
      }
    }
    // catch exception
    catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Serial port exception: %s", ex.what());
      is_usb_ok_ = false;

      try {
        if (serial_driver_ && serial_driver_->port() && serial_driver_->port()->is_open()) {
          serial_driver_->port()->close();
          RCLCPP_WARN(get_logger(), "Serial port closed due to exception");
        }
      } catch (const std::exception & close_ex) {
        RCLCPP_ERROR(get_logger(), "Failed to close serial port: %s", close_ex.what());
        // serial_driver_.reset();
      }
    }
    // sleep for a while before next check
    std::this_thread::sleep_for(std::chrono::milliseconds(USB_PROTECT_SLEEP_TIME));
  }

  // close serial port on exit
  RCLCPP_INFO(get_logger(), "serialPortProtect exit, close serial port");
  try {
    if (serial_driver_ && serial_driver_->port() && serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to close serial port on exit: %s", ex.what());
  }
  is_usb_ok_ = false;
}



/********************************************************/
/* Receive data                                         */
/********************************************************/

void StandardRobotPpRos2Node::receiveData()
{
  RCLCPP_INFO(get_logger(), "Start receiveData!");

  std::vector<uint8_t> sof(1);

  int time_waiting = 0;

  while (rclcpp::ok()) {
    // 串口状态
    if (!is_usb_ok_) {
      RCLCPP_WARN(get_logger(), "Receive: Usb is not ok! Wait for : %d s", time_waiting++);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;
    }

    try {
      serial_driver_->port()->receive(sof);

      if (sof[0] != SOF_HEAD) {
        continue;
      }

      // 根据数据段长度读取数据
      std::vector<uint8_t> data_buf(PACKAGE_LENGTH - 1);  // data_len
      int received_len = serial_driver_->port()->receive(data_buf);
      int received_len_sum = received_len;
      // 考虑到一次性读取数据可能存在数据量过大，读取不完整的情况。需要检测是否读取完整
      // 计算剩余未读取的数据长度
      int remain_len = PACKAGE_LENGTH - 1 - received_len;
      while (remain_len > 0) {  // 读取剩余未读取的数据
        std::vector<uint8_t> remain_buf(remain_len);
        received_len = serial_driver_->port()->receive(remain_buf);
        data_buf.insert(data_buf.begin() + received_len_sum, remain_buf.begin(), remain_buf.end());
        received_len_sum += received_len;
        remain_len -= received_len;
      }

      // 数据段读取完成后添加 header_frame_buf 到 data_buf，得到完整数据包
      data_buf.insert(data_buf.begin(), sof[0]);

      if(data_buf[PACKAGE_LENGTH - 3] != SOF_TAIL) {
        RCLCPP_ERROR(get_logger(), "Data tail error!");
        continue;
      }

      // 整包数据校验
      bool check_sum16_ok = checksum::verify_check_sum16(data_buf);
      if (!check_sum16_ok) {
        RCLCPP_ERROR(get_logger(), "Data segment check sum error!");
        continue;
      }
            
      float current_receive_time_ = fromVector<float>(data_buf, 2);
      float dt = current_receive_time_ - pkg_last_receive_time_;
      if (dt <= RECEIVE_TIMEOUT) {
        RCLCPP_WARN(get_logger(), "Receive data timeout! dt: %.2f s", dt);
        continue;
      }
      pkg_last_receive_time_ = current_receive_time_;

      switch(data_buf[1]){
        case RECEIVE_VISION_ID: {
          GimbalToVision gimbal_to_vision = fromVector<GimbalToVision>(data_buf);

          publish(gimbal_to_vision.yaw, gimbal_to_vision.pitch);
          pkg_last_receive_time_ = gimbal_to_vision.DWT_stamp;
          break;
        }
        case RECEIVE_REFEREE1_ID: {
          RefereePackage1 referee_package1 = fromVector<RefereePackage1>(data_buf);

          publish(referee_package1.game_status_data);
          publish(referee_package1.event_data);
          publish(referee_package1.robot_status_data);
          publish(referee_package1.hurt_data);
          publish(referee_package1.rfid_status_data);
          pkg_last_receive_time_ = referee_package1.DWT_stamp;
          break;
        }
        case RECEIVE_REFEREE2_ID: {
          RefereePackage2 referee_package2 = fromVector<RefereePackage2>(data_buf);

          publish(referee_package2.robot_pos_data);
          publish(referee_package2.ground_robot_pos_data);
          publish(referee_package2.game_robot_hp_data);
          pkg_last_receive_time_ = referee_package2.DWT_stamp;
          break;
        }
        default:
          RCLCPP_ERROR(get_logger(), "Unknown data package received! ID: %d", data_buf[1]);
          continue;
      }
      last_receive_time_ = this->now();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error receiving data: %s", ex.what());
      is_usb_ok_ = false;
      try {
        if (serial_driver_ && serial_driver_->port() && serial_driver_->port()->is_open()) {
          serial_driver_->port()->close();
        }
      } catch (const std::exception & close_ex) {
        RCLCPP_ERROR(get_logger(), "Close serial port after receive error failed: %s", close_ex.what());
      }
    }
  }
}

void StandardRobotPpRos2Node::publish(const GameStatusPackage::data & pkg)
{
  combat_rm_interfaces::msg::GameStatus msg;

  msg.game_progress     = pkg.game_progress;
  msg.stage_remain_time = pkg.stage_remain_time;

  game_status_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publish(const GameRobotHpPackage::data & pkg)
{
  combat_rm_interfaces::msg::GameRobotHp msg;

  msg.hero_hp         = pkg.hero_hp;
  msg.engineer_hp     = pkg.engineer_hp;
  msg.standard_3_hp   = pkg.standard_3_hp;
  msg.standard_4_hp   = pkg.standard_4_hp;
  msg.ally_outpost_hp = pkg.ally_outpost_hp;
  msg.ally_base_hp    = pkg.ally_base_hp;

  game_robot_hp_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publish(const EventDataPackage::data & pkg)
{
  combat_rm_interfaces::msg::EventData msg;

  msg.ally_supply_zone_non_exchange = pkg.ally_supply_zone_non_exchange;
  msg.ally_supply_zone_exchange     = pkg.ally_supply_zone_exchange;
  msg.ally_supply_zone              = pkg.ally_supply_zone;
  msg.ally_small_power_rune         = pkg.ally_small_power_rune;
  msg.ally_big_power_rune           = pkg.ally_big_power_rune;
  msg.central_highland              = pkg.central_highland;
  msg.trapezoidal_highland          = pkg.trapezoidal_highland;
  msg.center_gain_point             = pkg.center_gain_point;
  msg.ally_fortress_gain_point      = pkg.ally_fortress_gain_point;
  msg.ally_outpost_gain_point       = pkg.ally_outpost_gain_point;
  msg.base_gain_point               = pkg.base_gain_point;

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
    
    msg.armor_id            = pkg.armor_id;
    msg.hp_deduction_reason = pkg.hp_deduction_reason;

    hurt_data_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publish(const RfidStatusPackage::data & pkg)
{
  combat_rm_interfaces::msg::RfidStatus msg;

  msg.ally_base_gain_point              = pkg.ally_base_gain_point;
  msg.ally_central_highland_gain_point  = pkg.ally_central_highland_gain_point;
  msg.enemy_central_highland_gain_point = pkg.enemy_central_highland_gain_point;
  msg.ally_fortress_gain_point          = pkg.ally_fortress_gain_point;
  msg.ally_outpost_gain_point           = pkg.ally_outpost_gain_point;
  msg.ally_supply_point_non_exchange    = pkg.ally_supply_point_non_exchange;
  msg.ally_supply_point_exchange        = pkg.ally_supply_point_exchange;
  msg.center_gain_point                 = pkg.center_gain_point;
  msg.enemy_fortress_gain_point         = pkg.enemy_fortress_gain_point;
  msg.enemy_outpost_gain_point          = pkg.enemy_outpost_gain_point;

  rfid_status_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publish(const GroundRobotPositionPackage::data & pkg)
{
  combat_rm_interfaces::msg::GroundRobotPosition msg;

  msg.hero_position.x        = pkg.hero_x;
  msg.hero_position.y        = pkg.hero_y;
  msg.engineer_position.x    = pkg.engineer_x;
  msg.engineer_position.y    = pkg.engineer_y;
  msg.standard_3_position.x  = pkg.standard_3_x;
  msg.standard_3_position.y  = pkg.standard_3_y;
  msg.standard_4_position.x  = pkg.standard_4_x;
  msg.standard_4_position.y  = pkg.standard_4_y;

  ground_robot_position_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publish(float yaw, float pitch)
{
  // base yaw to odom_vision
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->now();
  t.header.frame_id = "base_yaw_odom";
  t.child_frame_id = "odom_vision";
  tf2::Quaternion q1, q2, q;
  q1.setRPY(0.0, 0.0, yaw);
  q2.setRPY(0.0, pitch, 0.0);
  tf2::Vector3 trans1(0.0, 0.0, 0.193);
  tf2::Vector3 trans2(0.0, 0.0, 0.11035);
  tf2::Vector3 trans3(0.078, 0.0, 0.0);
  tf2::Vector3 trans_total =
    trans1 + (tf2::quatRotate(q1, trans2)) + (tf2::quatRotate(q1 * q2, trans3));
  t.transform.translation = tf2::toMsg(trans_total);
  q.setRPY(0.0, 0.0, 0.0);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

/********************************************************/
/* Send data                                            */
/********************************************************/
void StandardRobotPpRos2Node::sendData()
{
  // rclcpp::Time current_time = this->now();
  RCLCPP_INFO(get_logger(), "Start send Data!");

  nav_to_gimbal_data_.head[0]         = SOF_HEAD;
  nav_to_gimbal_data_.head[1]         = 0x02;
  nav_to_gimbal_data_.tail            = SOF_TAIL;
  nav_to_gimbal_data_.check_sum       = 0;
  nav_to_gimbal_data_.time_stamp      = 0;
  nav_to_gimbal_data_.chassis_status  = 0;
  nav_to_gimbal_data_.sentry_status   = 0;
  nav_to_gimbal_data_.mode            = 0.0;
  nav_to_gimbal_data_.vx              = 0.0;
  nav_to_gimbal_data_.vy              = 0.0;
  nav_to_gimbal_data_.vyaw            = 0.0;
  
  int retry_count = 0;

  while (rclcpp::ok()) {
    if (!is_usb_ok_) {
      RCLCPP_WARN(get_logger(), "send: usb is not ok! Retry count: %d", retry_count++);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;
    }

    try {
      checksum::append_check_sum(
        reinterpret_cast<uint8_t *>(&nav_to_gimbal_data_), sizeof(NavToGimbal));
      nav_to_gimbal_data_.time_stamp = static_cast<uint64_t>(this->get_clock()->now().nanoseconds());
      // 发送数据
      std::vector<uint8_t> send_data = toVector(nav_to_gimbal_data_);
      serial_driver_->port()->send(send_data);
      retry_count = 0;
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error sending data: %s", ex.what());
      is_usb_ok_ = false;
      try {
        if (serial_driver_ && serial_driver_->port() && serial_driver_->port()->is_open()) {
          serial_driver_->port()->close();
        }
      } catch (const std::exception & close_ex) {
        RCLCPP_ERROR(get_logger(), "Close serial port after send error failed: %s", close_ex.what());
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

void StandardRobotPpRos2Node::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  nav_k_ = this->get_parameter("nav_k").as_double();
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
