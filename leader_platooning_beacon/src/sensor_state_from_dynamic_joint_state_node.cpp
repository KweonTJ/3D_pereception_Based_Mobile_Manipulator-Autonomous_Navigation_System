// Copyright 2026 ktj
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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlebot3_msgs/msg/sensor_state.hpp"

using namespace std::chrono_literals;

class SensorStateFromDynamicJointState : public rclcpp::Node
{
public:
  SensorStateFromDynamicJointState()
  : Node("sensor_state_from_dynamic_joint_state")
  {
    const auto input_topic = declare_parameter<std::string>(
      "dynamic_joint_states_topic", "/dynamic_joint_states");
    const auto output_topic = declare_parameter<std::string>(
      "sensor_state_topic", "/sensor_state");
    battery_sensor_name_ = declare_parameter<std::string>("battery_sensor_name", "battery");
    left_wheel_name_ = declare_parameter<std::string>("left_wheel_name", "wheel_left_joint");
    right_wheel_name_ = declare_parameter<std::string>("right_wheel_name", "wheel_right_joint");
    publish_period_s_ = declare_parameter<double>("publish_period_s", 0.1);
    encoder_ticks_per_revolution_ = declare_parameter<double>("encoder_ticks_per_revolution", 4096.0);

    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
    sensor_pub_ = create_publisher<turtlebot3_msgs::msg::SensorState>(output_topic, pub_qos);

    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    dynamic_state_sub_ = create_subscription<control_msgs::msg::DynamicJointState>(
      input_topic,
      sub_qos,
      std::bind(&SensorStateFromDynamicJointState::dynamicStateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "sensor_state relay started: %s -> %s",
      input_topic.c_str(),
      output_topic.c_str());
  }

private:
  static std::optional<double> findInterfaceValue(
    const control_msgs::msg::InterfaceValue & interfaces,
    const std::string & name)
  {
    const auto it = std::find(
      interfaces.interface_names.begin(),
      interfaces.interface_names.end(),
      name);
    if (it == interfaces.interface_names.end()) {
      return std::nullopt;
    }

    const auto index = static_cast<size_t>(std::distance(interfaces.interface_names.begin(), it));
    if (index >= interfaces.values.size() || !std::isfinite(interfaces.values[index])) {
      return std::nullopt;
    }
    return interfaces.values[index];
  }

  static std::optional<const control_msgs::msg::InterfaceValue *> findJointInterfaces(
    const control_msgs::msg::DynamicJointState & msg,
    const std::string & name)
  {
    const auto it = std::find(msg.joint_names.begin(), msg.joint_names.end(), name);
    if (it == msg.joint_names.end()) {
      return std::nullopt;
    }

    const auto index = static_cast<size_t>(std::distance(msg.joint_names.begin(), it));
    if (index >= msg.interface_values.size()) {
      return std::nullopt;
    }
    return &msg.interface_values[index];
  }

  int32_t radiansToEncoderTicks(const double radians) const
  {
    if (!std::isfinite(radians) || encoder_ticks_per_revolution_ <= 0.0) {
      return 0;
    }

    const auto ticks = radians * encoder_ticks_per_revolution_ / (2.0 * M_PI);
    return static_cast<int32_t>(std::llround(ticks));
  }

  std::optional<double> findValue(
    const control_msgs::msg::DynamicJointState & msg,
    const std::string & name,
    const std::string & interface_name) const
  {
    const auto interfaces = findJointInterfaces(msg, name);
    if (!interfaces) {
      return std::nullopt;
    }
    return findInterfaceValue(**interfaces, interface_name);
  }

  void dynamicStateCallback(const control_msgs::msg::DynamicJointState::SharedPtr msg)
  {
    const auto now = get_clock()->now();
    if (last_publish_time_.nanoseconds() > 0 &&
      (now - last_publish_time_).seconds() < publish_period_s_)
    {
      return;
    }

    turtlebot3_msgs::msg::SensorState sensor;
    sensor.header = msg->header;

    sensor.bumper = 0;
    sensor.cliff = 0.0f;
    sensor.sonar = 0.0f;
    sensor.illumination = 0.0f;
    sensor.led = 0;
    sensor.button = 0;
    sensor.torque = true;
    sensor.left_encoder = radiansToEncoderTicks(
      findValue(*msg, left_wheel_name_, "position").value_or(0.0));
    sensor.right_encoder = radiansToEncoderTicks(
      findValue(*msg, right_wheel_name_, "position").value_or(0.0));
    sensor.battery = static_cast<float>(
      findValue(*msg, battery_sensor_name_, "voltage").value_or(
        std::numeric_limits<float>::quiet_NaN()));

    sensor_pub_->publish(sensor);
    last_publish_time_ = now;
  }

  std::string battery_sensor_name_;
  std::string left_wheel_name_;
  std::string right_wheel_name_;
  double publish_period_s_;
  double encoder_ticks_per_revolution_;
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<turtlebot3_msgs::msg::SensorState>::SharedPtr sensor_pub_;
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr dynamic_state_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorStateFromDynamicJointState>());
  rclcpp::shutdown();
  return 0;
}
