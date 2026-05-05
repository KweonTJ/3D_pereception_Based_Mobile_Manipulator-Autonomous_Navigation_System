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

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class LeaderPlatooningBeacon : public rclcpp::Node
{
public:
  LeaderPlatooningBeacon()
  : Node("leader_platooning_beacon")
  {
    declare_parameter<double>("heartbeat_rate_hz", 10.0);
    declare_parameter<bool>("publish_odom_backup", false);
    declare_parameter<bool>("publish_cmd_vel_backup", false);
    declare_parameter<std::string>("odom_topic", "/odom");
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    declare_parameter<std::string>("heartbeat_topic", "/leader/heartbeat");
    declare_parameter<std::string>("leader_odom_topic", "/leader/odom");
    declare_parameter<std::string>("leader_cmd_vel_topic", "/leader/cmd_vel");

    const auto heartbeat_qos = rclcpp::QoS(rclcpp::KeepLast(3)).best_effort();
    heartbeat_pub_ = create_publisher<std_msgs::msg::Bool>(
      get_parameter("heartbeat_topic").as_string(), heartbeat_qos);

    double heartbeat_rate_hz = get_parameter("heartbeat_rate_hz").as_double();
    if (heartbeat_rate_hz <= 0.0) {
      RCLCPP_WARN(get_logger(), "heartbeat_rate_hz must be positive; using 10.0 Hz");
      heartbeat_rate_hz = 10.0;
    }
    const auto period = std::chrono::duration<double>(1.0 / heartbeat_rate_hz);
    heartbeat_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {
        publish_heartbeat();
      });

    if (get_parameter("publish_odom_backup").as_bool()) {
      const auto odom_qos = rclcpp::SensorDataQoS();
      leader_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        get_parameter("leader_odom_topic").as_string(), odom_qos);
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        get_parameter("odom_topic").as_string(), odom_qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          leader_odom_pub_->publish(*msg);
        });
      RCLCPP_INFO(get_logger(), "Leader odom backup relay enabled");
    }

    if (get_parameter("publish_cmd_vel_backup").as_bool()) {
      const auto cmd_vel_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
      leader_cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        get_parameter("leader_cmd_vel_topic").as_string(), cmd_vel_qos);
      cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        get_parameter("cmd_vel_topic").as_string(), cmd_vel_qos,
        [this](geometry_msgs::msg::Twist::SharedPtr msg) {
          leader_cmd_vel_pub_->publish(*msg);
        });
      RCLCPP_INFO(get_logger(), "Leader cmd_vel backup relay enabled");
    }

    RCLCPP_INFO(get_logger(), "Leader platooning heartbeat started");
  }

private:
  void publish_heartbeat()
  {
    std_msgs::msg::Bool msg;
    msg.data = true;
    heartbeat_pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr leader_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr leader_cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeaderPlatooningBeacon>());
  rclcpp::shutdown();
  return 0;
}
