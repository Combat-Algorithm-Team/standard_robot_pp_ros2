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

#ifndef STANDARD_ROBOT_PP_ROS2__STANDARD_ROBOT_PP_ROS2_HPP_
#define STANDARD_ROBOT_PP_ROS2__STANDARD_ROBOT_PP_ROS2_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "combat_rm_interfaces/msg/buff.hpp"
#include "combat_rm_interfaces/msg/event_data.hpp"
#include "combat_rm_interfaces/msg/game_robot_hp.hpp"
#include "combat_rm_interfaces/msg/game_status.hpp"
#include "combat_rm_interfaces/msg/ground_robot_position.hpp"
#include "combat_rm_interfaces/msg/hurt_data.hpp"
#include "combat_rm_interfaces/msg/rfid_status.hpp"
#include "combat_rm_interfaces/msg/robot_pos.hpp"
#include "combat_rm_interfaces/msg/robot_status.hpp"
#include "combat_rm_interfaces/msg/sentry_info.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "example_interfaces/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_utils/heartbeat.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "serial_driver/serial_driver.hpp"
#include "standard_robot_pp_ros2/packet_typedef.hpp"
// #include "combat_rm_interfaces/msg/navigation_cmd.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_interfaces/srv/set_mode.hpp"

namespace standard_robot_pp_ros2
{
class StandardRobotPpRos2Node : public rclcpp::Node
{
public:
  explicit StandardRobotPpRos2Node(const rclcpp::NodeOptions & options);

  ~StandardRobotPpRos2Node() override;

  struct SetModeClient
  {
    SetModeClient(rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr p) : ptr(p) {}
    std::atomic<bool> on_waiting = false;
    std::atomic<int> mode = 0;
    rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr ptr;
  };

private:
  // Parameter
  std::string device_name_;
  std::string vision_target_frame_;
  std::string base_link_frame_;
  float nav_k_;

  std::atomic<bool> is_usb_ok_{false};
  bool debug_;
  rclcpp::Time last_receive_time_;
  std::unique_ptr<IoContext> owned_ctx_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  bool record_rosbag_;
  bool set_detector_color_;

  // TF Broadcaster
  double timestamp_offset_ = 0;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::thread receive_thread_;
  std::thread send_thread_;
  std::thread serial_port_protect_thread_;

  // Publish
  rclcpp::Publisher<combat_rm_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
  rclcpp::Publisher<combat_rm_interfaces::msg::GameRobotHp>::SharedPtr game_robot_hp_pub_;
  rclcpp::Publisher<combat_rm_interfaces::msg::EventData>::SharedPtr event_data_pub_;
  rclcpp::Publisher<combat_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<combat_rm_interfaces::msg::RobotPos>::SharedPtr robot_pos_pub_;
  rclcpp::Publisher<combat_rm_interfaces::msg::Buff>::SharedPtr buff_pub_;
  rclcpp::Publisher<combat_rm_interfaces::msg::HurtData>::SharedPtr hurt_data_pub_;
  rclcpp::Publisher<combat_rm_interfaces::msg::RfidStatus>::SharedPtr rfid_status_pub_;
  rclcpp::Publisher<combat_rm_interfaces::msg::GroundRobotPosition>::SharedPtr
    ground_robot_position_pub_;
  rclcpp::Publisher<combat_rm_interfaces::msg::SentryInfo>::SharedPtr sentry_info_pub_;
  // vision publisher
  rclcpp::Publisher<rm_interfaces::msg::SerialReceiveData>::SharedPtr vision_data_pub_;
  fyt::HeartBeatPublisher::SharedPtr heartbeat_;

  // Subscribe
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<example_interfaces::msg::UInt8>::SharedPtr cmd_chassis_status_sub_;
  rclcpp::Subscription<example_interfaces::msg::UInt8>::SharedPtr cmd_sentry_status_sub_;
  rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr armor_solver_sub_;
  rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr rune_solver_sub_;

  // Client
  std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> getClients(
    rclcpp::Node * node) const;

  std::unordered_map<std::string, SetModeClient> set_mode_clients_;

  std::unordered_map<std::string, rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr>
    debug_pub_map_;

  SendTestData send_test_data_;

  void getParams();
  void createPublisher();
  void createSubscription();
  void receiveData();
  void sendData();
  void serialPortProtect();

  void publishGameStatus(const GameStatusPackage::data & pkg);
  void publishEventData(const EventDataPackage::data & pkg);
  void publishRobotStatus(const RobotStatusPackage::data & pkg);
  void publishHurtData(const HurtDataPackage::data & pkg);
  void publishRfidStatus(const RfidStatusPackage::data & pkg);
  void publishVisionData(const VisionDataPackage::data & pkg);

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void cmdChassisStatusCallback(example_interfaces::msg::UInt8::SharedPtr msg);
  void cmdSentryStatusCallback(example_interfaces::msg::UInt8::SharedPtr msg);
  void VisionCmdCallback(const rm_interfaces::msg::GimbalCmd::SharedPtr msg);

  bool callTriggerService(const std::string & service_name);
  void setMode(SetModeClient & client, const uint8_t mode);
};
}  // namespace standard_robot_pp_ros2

#endif  // STANDARD_ROBOT_PP_ROS2__STANDARD_ROBOT_PP_ROS2_HPP_
