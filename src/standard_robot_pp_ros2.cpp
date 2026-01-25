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

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Param client
  for (auto client : getClients(this)) {
    std::string name = client->get_service_name();
    set_mode_clients_.emplace(name, client);
    RCLCPP_INFO(get_logger(), "Create client for service: %s", name.c_str());
  }

  // Heartbeat
  heartbeat_ = fyt::HeartBeatPublisher::create(this);

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
    this->create_publisher<pb_rm_interfaces::msg::GameStatus>("referee/game_status", 10);
  rfid_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RfidStatus>("referee/rfid_status", 10);
  robot_status_pub_ =
    this->create_publisher<pb_rm_interfaces::msg::RobotStatus>("referee/robot_status", 10);
  serial_receive_data_pub_ = this->create_publisher<rm_interfaces::msg::SerialReceiveData>(
    "serial/receive", rclcpp::SensorDataQoS());
}

void StandardRobotPpRos2Node::createSubscription()
{
  chassis_cmd_sub_ = this->create_subscription<pb_rm_interfaces::msg::NavigationCmd>(
    "navigation_cmd", rclcpp::SensorDataQoS(),
    std::bind(&StandardRobotPpRos2Node::NavCmdCallback, this, std::placeholders::_1));

  armor_solver_sub_ = this->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "armor_solver/cmd_gimbal", rclcpp::SensorDataQoS(),
    std::bind(&StandardRobotPpRos2Node::VisionCmdCallback, this, std::placeholders::_1));

  rune_solver_sub_ = this->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "rune_solver/cmd_gimbal", rclcpp::SensorDataQoS(),
    std::bind(&StandardRobotPpRos2Node::VisionCmdCallback, this, std::placeholders::_1));
}

std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> StandardRobotPpRos2Node::getClients(
  rclcpp::Node * node) const {
  auto client1 = node->create_client<rm_interfaces::srv::SetMode>("armor_detector/set_mode",
                                                                  rmw_qos_profile_services_default);
  auto client2 = node->create_client<rm_interfaces::srv::SetMode>("armor_solver/set_mode",
                                                                  rmw_qos_profile_services_default);
  return {client1, client2};
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
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    vision_target_frame_ = declare_parameter<std::string>("vision_target_frame", "odom_vision");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The vision target frame provided was invalid");
    throw ex;
  }

  try {
    timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
    timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The timestamp offset provided was invalid");
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

  // @TODO: 1.保持串口连接 2.串口断开重连 3.串口异常处理

  // 初始化串口
  serial_driver_->init_port(device_name_, *device_config_);
  // 尝试打开串口
  try {
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      RCLCPP_WARN(get_logger(), "Serial port opened!");
      is_usb_ok_ = true;
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Open serial port failed : %s", ex.what());
    is_usb_ok_ = false;
  }

  is_usb_ok_ = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(USB_PROTECT_SLEEP_TIME));

  while (rclcpp::ok()) {
    if (!is_usb_ok_) {
      try {
        if (serial_driver_->port()->is_open()) {
          serial_driver_->port()->close();
        }

        serial_driver_->port()->open();

        if (serial_driver_->port()->is_open()) {
          RCLCPP_INFO(get_logger(), "Serial port opened!");
          is_usb_ok_ = true;
        }
      } catch (const std::exception & ex) {
        is_usb_ok_ = false;
        RCLCPP_ERROR(get_logger(), "Open serial port failed : %s", ex.what());
      }
    }

    // thread sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(USB_PROTECT_SLEEP_TIME));
  }
}

/********************************************************/
/* Receive data                                         */
/********************************************************/

void StandardRobotPpRos2Node::receiveData()
{
  RCLCPP_INFO(get_logger(), "Start receiveData!");

  std::vector<uint8_t> sof(1);

  int sof_count = 0;
  int retry_count = 0;
  int data_length = 64;

  while (rclcpp::ok()) {
    // 串口状态
    if (!is_usb_ok_) {
      RCLCPP_WARN(get_logger(), "receive: usb is not ok! Retry count: %d", retry_count++);
      std::this_thread::sleep_for(std::chrono::milliseconds(USB_NOT_OK_SLEEP_TIME));
      continue;
    }

    try {
      serial_driver_->port()->receive(sof);

      if (sof[0] != SOF_RECEIVE) {
        sof_count++;
        RCLCPP_INFO(get_logger(), "Finding sof, cnt=%d", sof_count);
        continue;
      }

      // Reset sof_count when SOF_RECEIVE is found
      sof_count = 0;
      int data_remain_length = data_length - 1;  // 已经读取了 sof，占用 1 字节

      // 根据数据段长度读取数据
      std::vector<uint8_t> data_buf(data_remain_length);  // len + csum
      int received_len = serial_driver_->port()->receive(data_buf);
      int received_len_sum = received_len;
      // 考虑到一次性读取数据可能存在数据量过大，读取不完整的情况。需要检测是否读取完整
      // 计算剩余未读取的数据长度
      int remain_len = data_length - received_len - 1;
      while (remain_len > 0) {  // 读取剩余未读取的数据
        std::vector<uint8_t> remain_buf(remain_len);
        received_len = serial_driver_->port()->receive(remain_buf);
        data_buf.insert(data_buf.begin() + received_len_sum, remain_buf.begin(), remain_buf.end());
        received_len_sum += received_len;
        remain_len -= received_len;
      }

      // 数据段读取完成后添加 header_frame_buf 到 data_buf，得到完整数据包
      data_buf.insert(data_buf.begin(), sof[0]);

      bool checksum_ok = checksum::verify_check_sum(data_buf);
      if (!checksum_ok && data_buf[63] != SOF_TAIL) {
        RCLCPP_ERROR(get_logger(), "Check sum or tail_frame 0x%x error! ", data_buf[63]);
      }

      ReceiveTestData received_test_data = fromVector<ReceiveTestData>(data_buf);
      processTestData(received_test_data);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Error receiving data: %s", ex.what());
      is_usb_ok_ = false;
    }
  }
}

void StandardRobotPpRos2Node::processTestData(ReceiveTestData & received_test_data)
{
  // 发布 serial_receive_data
  rm_interfaces::msg::SerialReceiveData serial_receive_data;
  serial_receive_data.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
  serial_receive_data.header.frame_id = vision_target_frame_;
  serial_receive_data.mode = received_test_data.data.enemy_color == 2 ? 1 : 0;
  serial_receive_data.bullet_speed = received_test_data.data.bullet_speed;
  serial_receive_data.roll = 0.0;
  serial_receive_data.pitch = received_test_data.data.pitch;
  serial_receive_data.yaw = received_test_data.data.yaw;
  serial_receive_data_pub_->publish(serial_receive_data);

  for (auto & [service_name, client] : set_mode_clients_) {
    if (client.mode.load() != serial_receive_data.mode && !client.on_waiting.load()) {
      setMode(client, serial_receive_data.mode);
    }
  }
  
  geometry_msgs::msg::TransformStamped t;
  timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
  t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
  t.header.frame_id = vision_target_frame_;
  t.child_frame_id = "gimbal_link";
  auto roll = 0.0;
  auto pitch = -received_test_data.data.pitch;
  auto yaw = received_test_data.data.yaw;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);

  // odom_rectify: 转了roll角后的坐标系
  Eigen::Quaterniond q_eigen(q.w(), q.x(), q.y(), q.z());
  Eigen::Vector3d rpy = fyt::utils::getRPY(q_eigen.toRotationMatrix());
  q.setRPY(rpy[0], 0, 0);
  t.header.frame_id = vision_target_frame_;
  t.child_frame_id = vision_target_frame_ + "_rectify";
  tf_broadcaster_->sendTransform(t);

  // base yaw to odom_vision
  t.header.frame_id = "base_yaw_odom";
  t.child_frame_id = "odom_vision";
  tf2::Quaternion q1, q2, q3, q_total;
  q1.setRPY(0.0, 0.0, yaw);
  q2.setRPY(0.0, pitch, 0.0);
  q3.setRPY(0.0, -pitch, -yaw);
  q_total = q1 * q2 * q3;
  tf2::Vector3 trans1(0.0, 0.0, 0.193);
  tf2::Vector3 trans2(0.0, 0.0, 0.11035);
  tf2::Vector3 trans3(0.078, 0.0, 0.0);
  tf2::Vector3 trans_total = trans1 + 
                          (tf2::quatRotate(q1, trans2)) + 
                          (tf2::quatRotate(q1 * q2, trans3));
  t.transform.translation = tf2::toMsg(trans_total);
  t.transform.rotation = tf2::toMsg(q_total);
  tf_broadcaster_->sendTransform(t);

  // 发布 robot_status
  pb_rm_interfaces::msg::RobotStatus robot_status_msg;
  robot_status_msg.robot_id = 7;
  robot_status_msg.robot_level = received_test_data.data.robot_level;
  robot_status_msg.current_hp = received_test_data.data.current_hp;
  robot_status_msg.maximum_hp = received_test_data.data.maximum_hp;
  robot_status_msg.armor_id = received_test_data.data.armor_id;
  robot_status_msg.hp_deduction_reason = received_test_data.data.hp_deduction_reason;
  robot_status_pub_->publish(robot_status_msg);

  // 发布 game_status
  pb_rm_interfaces::msg::GameStatus game_status_msg;
  game_status_msg.game_progress = received_test_data.data.game_progress;
  game_status_msg.stage_remain_time = received_test_data.data.stage_remain_time;
  game_status_pub_->publish(game_status_msg);

  // 发布 rfid_status
  pb_rm_interfaces::msg::RfidStatus rfid_status_msg;
  rfid_status_msg.base_gain_point = received_test_data.data.rfid_status != 0;
  rfid_status_msg.ally_central_highland_gain_point = (received_test_data.data.rfid_status & 1) != 0;
  rfid_status_msg.enemy_central_highland_gain_point = (received_test_data.data.rfid_status & (1 << 2)) != 0;
  rfid_status_msg.ally_fortress_gain_point = (received_test_data.data.rfid_status & (1 << 17)) != 0;
  rfid_status_msg.ally_outpost_gain_point = (received_test_data.data.rfid_status & (1 << 18)) != 0;
  rfid_status_msg.ally_supply_zone_non_exchange = (received_test_data.data.rfid_status & (1 << 19)) != 0;
  rfid_status_msg.ally_supply_zone_exchange = (received_test_data.data.rfid_status & (1 << 20)) != 0;
  rfid_status_msg.ally_assemble_island = (received_test_data.data.rfid_status & (1 << 21)) != 0;
  rfid_status_msg.enemy_assemble_island = (received_test_data.data.rfid_status & (1 << 22)) != 0;
  rfid_status_msg.center_gain_point = (received_test_data.data.rfid_status & (1 << 23)) != 0;
  rfid_status_msg.enemy_fortress_gain_point = (received_test_data.data.rfid_status & (1 << 24)) != 0;
  rfid_status_msg.enemy_outpost_gain_point = (received_test_data.data.rfid_status & (1 << 25)) != 0;
  uint32_t valid_bits = received_test_data.data.rfid_status & 0xFFFFFFFF;
  int set_bits_count = std::bitset<32>(valid_bits).count();
  if (set_bits_count == 1 || set_bits_count == 0) {
    rfid_status_pub_->publish(rfid_status_msg);
  }
  else {
    RCLCPP_WARN(get_logger(), "RFID status has multiple bits set, not publishing! The RFID Status = %x",
      received_test_data.data.rfid_status);
    return;
  }
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

void StandardRobotPpRos2Node::NavCmdCallback(const pb_rm_interfaces::msg::NavigationCmd::SharedPtr msg)
{
  send_test_data_.data.vx = msg->twist.linear.x;
  send_test_data_.data.vy = msg->twist.linear.y;

  send_test_data_.data.chassis_status = msg->chassis_status;
}

void StandardRobotPpRos2Node::VisionCmdCallback(const rm_interfaces::msg::GimbalCmd::SharedPtr msg)
{
  send_test_data_.data.fire_advice = msg->fire_advice;
  send_test_data_.data.pitch = msg->pitch/180.0f*M_PI;
  send_test_data_.data.yaw = msg->yaw;
  send_test_data_.data.sec = msg->header.stamp.sec;
  send_test_data_.data.nanosec = msg->header.stamp.nanosec;
  if (msg->distance < 0) {
    send_test_data_.data.major_number = 0;
  } else {
    send_test_data_.data.major_number = 1;
  } //TODO: more major number
}

void StandardRobotPpRos2Node::setMode(SetModeClient &client, const uint8_t mode) {
  using namespace std::chrono_literals;

  std::string service_name = client.ptr->get_service_name();
  // Wait for service
  while (!client.ptr->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        get_logger(), "Interrupted while waiting for the service %s. Exiting.", service_name.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "Service %s not available, waiting again...", service_name.c_str());
  }
  if (!client.ptr->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service: %s is not available!", service_name.c_str());
    return;
  }
  // Send request
  auto req = std::make_shared<rm_interfaces::srv::SetMode::Request>();
  req->mode = mode;

  client.on_waiting.store(true);
  auto result = client.ptr->async_send_request(
    req, [mode, &client](rclcpp::Client<rm_interfaces::srv::SetMode>::SharedFuture result) {
      client.on_waiting.store(false);
      if (result.get()->success) {
        client.mode.store(mode);
      }
    });
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
