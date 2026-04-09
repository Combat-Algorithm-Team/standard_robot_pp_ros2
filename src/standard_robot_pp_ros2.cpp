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

// #include "rm_utils/math/utils.hpp"
#include "standard_robot_pp_ros2/crc8_crc16.hpp"
#include "standard_robot_pp_ros2/packet_typedef.hpp"
#include "standard_robot_pp_ros2/crc8_crc16.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include "rm_utils/math/utils.hpp"

#define USB_NOT_OK_SLEEP_TIME 1000   // (ms)
#define USB_PROTECT_SLEEP_TIME 1000  // (ms)
#define LIVOX_IMU_HZ 200

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
  buff_pub_ =
    this->create_publisher<combat_rm_interfaces::msg::Buff>("referee/buff", 10);
  hurt_data_pub_ =
    this->create_publisher<combat_rm_interfaces::msg::HurtData>("referee/hurt_data", 10);
  rfid_status_pub_ =
    this->create_publisher<combat_rm_interfaces::msg::RfidStatus>("referee/rfid_status", 10);
  robot_status_pub_ =
    this->create_publisher<combat_rm_interfaces::msg::RobotStatus>("referee/robot_status", 10);
  ground_robot_position_pub_ =
    this->create_publisher<combat_rm_interfaces::msg::GroundRobotPosition>("referee/ground_robot_position", 10);
  sentry_info_pub_ =
    this->create_publisher<combat_rm_interfaces::msg::SentryInfo>("referee/sentry_info", 10);
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
    nav_k_ = declare_parameter<float>("nav_k", 1.0);
    nav_k_ = this->get_parameter("nav_k").as_float();
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

  // set_detector_color_ = declare_parameter("set_detector_color", false);
}

/********************************************************/
/* Serial port protect                                  */
/********************************************************/
void StandardRobotPpRos2Node::serialPortProtect()
{
  RCLCPP_INFO(get_logger(), "Start serialPortProtect!");

  while (rclcpp::ok()) {
    try {
      // 1. check if serial driver and port are valid and open
      if (!serial_driver_ || !serial_driver_->port() || !serial_driver_->port()->is_open()) {
        RCLCPP_WARN(get_logger(), "Serial port is not open, try to reconnect...");

        bool found_port = false;
        std::string base_port = device_name_;
        size_t last_non_digit = base_path.find_last_not_of("0123456789");
        if (last_non_digit != std::string::npos && last_non_digit + 1 < base_path.size()) {
            base_path = base_path.substr(0, last_non_digit + 1);  // 去掉末尾数字
        }
        for (int i = 0; i < 10; ++i) {
          std::string candidate = base_path + std::to_string(i);
          if (std::filesystem::exists(candidate)) {
              // 找到设备
              RCLCPP_INFO(get_logger(), "Serial File %s found!", candidate.c_str());
              device_name_ = candidate;
              found_port = true;
              break;
          }
        } else {
            RCLCPP_WARN(get_logger(), "Serial File %s not found! Trying next...", candidate.c_str());
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
          } else {
            RCLCPP_ERROR(get_logger(), "Serial port %s open failed (port is null or open failed)", device_name_.c_str());
            is_usb_ok_ = false;
          }
        }else {
          RCLCPP_ERROR(get_logger(), "No serial port found in range 0-9 with base %s", device_name_.c_str(), current_port.c_str());
          is_usb_ok_ = false;
        }
      } 
      // the serial port is open
      // TODO: 可以在这里添加额外的串口检查，例如发送心跳包或读取数据验证通信是否正常
      else {
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
        serial_driver_.reset();
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
  int sof_err_count = 0;
  bool is_checksum = false;

  while (rclcpp::ok()) {
    // 串口状态
    if (!is_usb_ok_) {
      RCLCPP_WARN(get_logger(), "Receive: Usb is not ok! Wait for : %d s", time_waiting++);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;
    }

    try {
      if (sof_err_count >= MAX_PACKAGE_LEN) {
        RCLCPP_WARN(get_logger(), "Receive: SOF error count over 64! SOF_HEAD not found. Data receive fail.");
        RCLCPP_WARN(get_logger(), "Set usb is not ok!");
        is_usb_ok_ = false;
        sof_err_count = 0;
      }

      serial_driver_->port()->receive(sof);
      is_checksum = true;

      if (sof[0] != 0x5A) {
        continue;
      }

      // Reset sof_count when SOF_HEAD is found
      sof_err_count = 0;

      // sof[0] == SOF_HEAD 后读取剩余 header_frame 内容
      std::vector<uint8_t> header_frame_buf(4);

      serial_driver_->port()->receive(header_frame_buf);  // 读取除 sof 外剩下的包头数据
      header_frame_buf.insert(header_frame_buf.begin(), sof[0]);  // 添加 sof
      HeaderFrame header_frame = fromVector<HeaderFrame>(header_frame_buf);

      // HeaderFrame CRC8/CHECK_SUM8 check
      if (!is_checksum) {
        bool crc8_ok = crc8::verify_CRC8_check_sum(
        reinterpret_cast<uint8_t *>(&header_frame), sizeof(header_frame));
        if (!crc8_ok) {
          RCLCPP_ERROR(get_logger(), "Header frame CRC8 error!");
          continue;
        }
      }else {
        bool check_sum8_ok = checksum::verify_check_sum8(
        reinterpret_cast<uint8_t *>(&header_frame), sizeof(header_frame));
        if (!check_sum8_ok) {
          RCLCPP_ERROR(get_logger(), "Header frame check sum error!");
          continue;
        }
      }
      
      // crc8_ok 校验正确后读取数据段
      // 根据数据段长度读取数据
      std::vector<uint8_t> data_buf(header_frame.len + 4);  // data_len + cmd_id + crc16
      int received_len = serial_driver_->port()->receive(data_buf);
      int received_len_sum = received_len;
      // 考虑到一次性读取数据可能存在数据量过大，读取不完整的情况。需要检测是否读取完整
      // 计算剩余未读取的数据长度
      int remain_len = header_frame.len + 2 - received_len;
      while (remain_len > 0) {  // 读取剩余未读取的数据
        std::vector<uint8_t> remain_buf(remain_len);
        received_len = serial_driver_->port()->receive(remain_buf);
        data_buf.insert(data_buf.begin() + received_len_sum, remain_buf.begin(), remain_buf.end());
        received_len_sum += received_len;
        remain_len -= received_len;
      }

      // 数据段读取完成后添加 header_frame_buf 到 data_buf，得到完整数据包
      data_buf.insert(data_buf.begin(), header_frame_buf.begin(), header_frame_buf.end());

      // 整包数据校验
      bool check_sum16_ok = checksum::verify_check_sum16(data_buf);
      if (!check_sum16_ok) {
        RCLCPP_ERROR(get_logger(), "Data segment check sum error!");
        continue;
      }
      last_receive_time_ = this->now();

      ReceiveDataPackage receive_data_package = fromVector<ReceiveDataPackage>(data_buf);

      publishVisionData(receive_data_package.data.vision_data);
      publishGameStatus(receive_data_package.data.game_status_data);
      publishEventData(receive_data_package.data.event_data);
      publishRobotStatus(receive_data_package.data.robot_status_data);
      publishHurtData(receive_data_package.data.hurt_data);
      publishRfidStatus(receive_data_package.data.rfid_data);

    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error receiving data: %s", ex.what());
      is_usb_ok_ = false;
    }
  }
}

void StandardRobotPpRos2Node::publishGameStatus(const GameStatusPackage& pkg)
{
  combat_rm_interfaces::msg::GameStatus msg;

  msg.game_progress = pkg.data.game_progress;
  msg.stage_remain_time = pkg.data.stage_remain_time;
  msg.sync_time_stamp = pkg.data.sync_time_stamp;

  game_status_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishGameResult(const GameResultPackage& pkg)
{
    // Function content reserved for user implementation
    return;
}

void StandardRobotPpRos2Node::publishGameRobotHp(const GameRobotHpPackage& pkg)
{
  combat_rm_interfaces::msg::GameRobotHp msg;

  msg.ally_1_robot_hp = pkg.data.ally_1_robot_hp;
  msg.ally_2_robot_hp = pkg.data.ally_2_robot_hp;
  msg.ally_3_robot_hp = pkg.data.ally_3_robot_hp;
  msg.ally_4_robot_hp = pkg.data.ally_4_robot_hp;
  msg.ally_outpost_hp = pkg.data.ally_outpost_hp;
  msg.ally_base_hp = pkg.data.ally_base_hp;

  game_robot_hp_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishEventData(const EventDataPackage& pkg)
{
  combat_rm_interfaces::msg::EventData msg;

  msg.ally_supply_zone_non_exchange = pkg.data.ally_supply_zone_non_exchange;
  msg.ally_supply_zone_exchange     = pkg.data.ally_supply_zone_exchange;
  msg.ally_supply_zone              = pkg.data.ally_supply_zone;
  msg.ally_small_power_rune         = pkg.data.ally_small_power_rune;
  msg.ally_big_power_rune           = pkg.data.ally_big_power_rune;
  msg.central_highland              = pkg.data.central_highland;
  msg.trapezoidal_highland          = pkg.data.trapezoidal_highland;
  msg.center_gain_point              = pkg.data.center_gain_point;
  msg.ally_fortress_gain_point      = pkg.data.ally_fortress_gain_point;
  msg.ally_outpost_gain_point       = pkg.data.ally_outpost_gain_point;
  msg.base_gain_point               = pkg.data.base_gain_point;

  event_data_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishRefreeWarnning(const RefreeWarnningPackage& pkg)
{
  // Function content reserved for user implementation
  return;
}

void StandardRobotPpRos2Node::publishDartInfo(const DartInfoPackage& pkg)
{
  // Function content reserved for user implementation
  return;
}

void StandardRobotPpRos2Node::publishRobotStatus(const RobotStatusPackage& pkg)
{
  combat_rm_interfaces::msg::RobotStatus msg;

  msg.current_hp = pkg.data.current_hp;
  msg.maximum_hp = pkg.data.maximum_hp;

  robot_status_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishPowerHeatData(const PowerHeatDataPackage& pkg)
{
    // Function content reserved for user implementation
    return;
}

void StandardRobotPpRos2Node::publishRobotPos(const RobotPosPackage& pkg)
{
  combat_rm_interfaces::msg::RobotPos msg;

  msg.x = pkg.data.x;
  msg.y = pkg.data.y;
  msg.angle = pkg.data.angle;

  robot_pos_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishBuff(const BuffPackage& pkg)
{
  combat_rm_interfaces::msg::Buff msg;

  msg.recovery_buff = pkg.data.recovery_buff;
  msg.cooling_buff = pkg.data.cooling_buff;
  msg.defence_buff = pkg.data.defence_buff;
  msg.vulnerability_buff = pkg.data.vulnerability_buff;
  msg.attack_buff = pkg.data.attack_buff;
  msg.remaining_energy = pkg.data.remaining_energy;

  buff_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishHurtData(const HurtDataPackage& pkg)
{
    combat_rm_interfaces::msg::HurtData msg;
    
    msg.armor_id = pkg.data.armor_id;
    msg.hp_deduction_reason = pkg.data.hp_deduction_reason;

    hurt_data_pub_->publish(msg);
}

void StandardRobotPpRos2Node::publishShootData(const ShootDataPackage& pkg)
{
    // Function content reserved for user implementation
    return;
}

void StandardRobotPpRos2Node::publishProjectileAllowance(const ProjectileAllowancePackage& pkg)
{
    // Function content reserved for user implementation
    return;
}

void StandardRobotPpRos2Node::publishRfidStatus(const RfidStatusPackage& pkg)
{
  combat_rm_interfaces::msg::RfidStatus msg;

  msg.ally_base_gain_point = pkg.data.ally_base_gain_point;
  msg.ally_central_highland_gain_point = pkg.data.ally_central_highland_gain_point;
  msg.enemy_central_highland_gain_point = pkg.data.enemy_central_highland_gain_point;
  msg.ally_fortress_gain_point = pkg.data.ally_fortress_gain_point;
  msg.ally_outpost_gain_point = pkg.data.ally_outpost_gain_point;
  msg.ally_supply_point_non_exchange = pkg.data.ally_supply_point_non_exchange;
  msg.ally_supply_point_exchange = pkg.data.ally_supply_point_exchange;
  msg.center_gain_point = pkg.data.center_gain_point;
  msg.enemy_fortress_gain_point = pkg.data.enemy_fortress_gain_point;
  msg.enemy_outpost_gain_point = pkg.data.enemy_outpost_gain_point;

  rfid_status_pub_->publish(msg);
}

/********************************************************/
/* Send data                                            */
/********************************************************/
void StandardRobotPpRos2Node::sendData()
{
  // rclcpp::Time current_time = this->now();
  RCLCPP_INFO(get_logger(), "Start send Data!");

  send_test_data_.frame_header = SOF_SEND;
  send_test_data_.frame_tail = SOF_TAIL;
  send_test_data_.check_sum = 0;
  send_test_data_.data.fire_advice = 0;
  send_test_data_.data.major_number = 0;
  send_test_data_.data.chassis_status = 0;
  send_test_data_.data.pitch = 0.0;
  send_test_data_.data.yaw = 0.0;
  send_test_data_.data.sec = 0;
  send_test_data_.data.nanosec = 0.0;
  send_test_data_.data.vx = 0.0;
  send_test_data_.data.vy = 0.0;
  
  int retry_count = 0;

  while (rclcpp::ok()) {
    if (!is_usb_ok_) {
      RCLCPP_WARN(get_logger(), "send: usb is not ok! Retry count: %d", retry_count++);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;
    }

    try {
      checksum::append_check_sum(
        reinterpret_cast<uint8_t *>(&send_test_data_), sizeof(SendTestData));
      // 发送数据
      std::vector<uint8_t> send_data = toVector(send_test_data_);
      serial_driver_->port()->send(send_data);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error sending data: %s", ex.what());
      is_usb_ok_ = false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void StandardRobotPpRos2Node::NavCmdCallback(const combat_rm_interfaces::msg::NavigationCmd::SharedPtr msg)
{
  send_test_data_.data.vx = msg->twist.linear.x * nav_k_;
  send_test_data_.data.vy = msg->twist.linear.y * nav_k_;
  send_test_data_.data.chassis_status = msg->chassis_status;
}

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
