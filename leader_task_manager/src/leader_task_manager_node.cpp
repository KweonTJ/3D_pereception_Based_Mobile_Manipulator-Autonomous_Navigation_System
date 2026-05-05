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
#include <cctype>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class LeaderTaskManager : public rclcpp::Node
{
public:
  LeaderTaskManager()
  : Node("leader_task_manager")
  {
    declare_parameter<double>("publish_rate_hz", 10.0);
    declare_parameter<std::string>("mp_control_status_topic", "/mp_control/status");
    declare_parameter<std::string>("cargo_events_topic", "/cargo/events");
    declare_parameter<std::string>("cargo_current_id_topic", "/cargo/current_id");
    declare_parameter<std::string>("task_state_topic", "/leader/task_state");
    declare_parameter<std::string>("cargo_state_topic", "/leader/cargo_state");
    declare_parameter<std::string>("follower_enable_topic", "/leader/follower_enable");
    declare_parameter<std::string>("platoon_mode_topic", "/leader/platoon_mode");
    declare_parameter<std::string>("initial_task_state", "IDLE");
    declare_parameter<std::string>("initial_cargo_state", "EMPTY");
    declare_parameter<std::string>("initial_platoon_mode", "STOP");
    declare_parameter<bool>("initial_follower_enable", false);

    task_state_ = get_parameter("initial_task_state").as_string();
    cargo_state_ = get_parameter("initial_cargo_state").as_string();
    platoon_mode_ = get_parameter("initial_platoon_mode").as_string();
    follower_enable_ = get_parameter("initial_follower_enable").as_bool();

    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    task_state_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("task_state_topic").as_string(), state_qos);
    cargo_state_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("cargo_state_topic").as_string(), state_qos);
    follower_enable_pub_ = create_publisher<std_msgs::msg::Bool>(
      get_parameter("follower_enable_topic").as_string(), state_qos);
    platoon_mode_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("platoon_mode_topic").as_string(), state_qos);

    mp_control_status_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("mp_control_status_topic").as_string(), 10,
      [this](std_msgs::msg::String::SharedPtr msg) {
        handle_mp_control_status(msg->data);
      });
    cargo_events_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("cargo_events_topic").as_string(), 10,
      [this](std_msgs::msg::String::SharedPtr msg) {
        handle_cargo_event(msg->data);
      });
    cargo_current_id_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("cargo_current_id_topic").as_string(), 10,
      [this](std_msgs::msg::String::SharedPtr msg) {
        current_cargo_id_ = msg->data;
      });

    double publish_rate_hz = get_parameter("publish_rate_hz").as_double();
    if (publish_rate_hz <= 0.0) {
      RCLCPP_WARN(get_logger(), "publish_rate_hz must be positive; using 10.0 Hz");
      publish_rate_hz = 10.0;
    }
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {
        publish_states();
      });

    RCLCPP_INFO(
      get_logger(),
      "Leader task manager started: task=%s cargo=%s follower_enable=%s platoon=%s",
      task_state_.c_str(), cargo_state_.c_str(), follower_enable_ ? "true" : "false",
      platoon_mode_.c_str());
  }

private:
  static std::string uppercase(std::string text)
  {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
      return static_cast<char>(std::toupper(c));
    });
    return text;
  }

  static bool contains(const std::string & haystack, const std::string & needle)
  {
    return haystack.find(needle) != std::string::npos;
  }

  static bool contains_any(
    const std::string & haystack, const std::vector<std::string> & needles)
  {
    return std::any_of(needles.begin(), needles.end(), [&](const auto & needle) {
      return contains(haystack, needle);
    });
  }

  void handle_mp_control_status(const std::string & raw_status)
  {
    const auto status = uppercase(raw_status);

    if (contains_any(status, {"ERROR", "FAIL"})) {
      set_task_state("ERROR");
      set_cargo_state("ERROR");
      set_follower_enable(false);
      set_platoon_mode("STOP");
      return;
    }

    if (contains_any(status, {"GRASPED", "PICK_SUCCESS", "PICK_DONE", "PICKED"})) {
      set_cargo_state("GRASPED");
      set_task_state("WAIT_FOLLOWER");
      set_follower_enable(true);
      set_platoon_mode("STANDBY");
      return;
    }

    if (contains_any(status, {"PLACE_DONE", "LOAD_DONE", "LOADED"})) {
      set_task_state("CARGO_LOADED");
      set_cargo_state("LOADED");
      set_follower_enable(true);
      set_platoon_mode("FOLLOW");
      return;
    }

    if (contains(status, "DONE")) {
      set_task_state("DONE");
      set_follower_enable(false);
      set_platoon_mode("STOP");
      return;
    }

    if (contains_any(status, {"PLACING", "PLACE"})) {
      set_task_state("PLACING_ON_FOLLOWER");
      set_cargo_state("LOADING");
      set_follower_enable(true);
      set_platoon_mode("STANDBY");
      return;
    }

    if (contains_any(status, {"PICKING", "GRASPING", "PICK:"})) {
      set_task_state("PICKING");
      return;
    }

    if (contains_any(status, {"MOVING", "NAVIGATING"})) {
      set_task_state("MOVING");
      set_follower_enable(true);
      set_platoon_mode("FOLLOW");
      return;
    }

    RCLCPP_WARN(get_logger(), "Unknown mp_control status ignored: '%s'", raw_status.c_str());
  }

  void handle_cargo_event(const std::string & raw_event)
  {
    const auto event = uppercase(raw_event);

    if (contains_any(event, {"ERROR", "FAIL"})) {
      set_cargo_state("ERROR");
      set_task_state("ERROR");
      set_follower_enable(false);
      set_platoon_mode("STOP");
      return;
    }

    if (contains_any(event, {"GRASPED", "PICKED"})) {
      set_cargo_state("GRASPED");
      return;
    }

    if (contains_any(event, {"LOADED", "LOAD_DONE", "PLACED"})) {
      set_cargo_state("LOADED");
      set_task_state("CARGO_LOADED");
      set_follower_enable(true);
      set_platoon_mode("FOLLOW");
      return;
    }

    if (contains(event, "DELIVERED")) {
      set_cargo_state("DELIVERED");
      set_task_state("DONE");
      set_follower_enable(false);
      set_platoon_mode("STOP");
      return;
    }

    RCLCPP_WARN(get_logger(), "Unknown cargo event ignored: '%s'", raw_event.c_str());
  }

  void publish_states()
  {
    std_msgs::msg::String task_msg;
    task_msg.data = task_state_;
    task_state_pub_->publish(task_msg);

    std_msgs::msg::String cargo_msg;
    cargo_msg.data = cargo_state_;
    cargo_state_pub_->publish(cargo_msg);

    std_msgs::msg::Bool follower_msg;
    follower_msg.data = follower_enable_;
    follower_enable_pub_->publish(follower_msg);

    std_msgs::msg::String platoon_msg;
    platoon_msg.data = platoon_mode_;
    platoon_mode_pub_->publish(platoon_msg);
  }

  void set_task_state(const std::string & value)
  {
    set_string_state("task_state", task_state_, value);
  }

  void set_cargo_state(const std::string & value)
  {
    set_string_state("cargo_state", cargo_state_, value);
  }

  void set_platoon_mode(const std::string & value)
  {
    set_string_state("platoon_mode", platoon_mode_, value);
  }

  void set_string_state(
    const std::string & name, std::string & field, const std::string & value)
  {
    if (field == value) {
      return;
    }
    RCLCPP_INFO(get_logger(), "%s: %s -> %s", name.c_str(), field.c_str(), value.c_str());
    field = value;
  }

  void set_follower_enable(bool value)
  {
    if (follower_enable_ == value) {
      return;
    }
    RCLCPP_INFO(
      get_logger(), "follower_enable: %s -> %s",
      follower_enable_ ? "true" : "false", value ? "true" : "false");
    follower_enable_ = value;
  }

  std::string task_state_;
  std::string cargo_state_;
  std::string platoon_mode_;
  std::string current_cargo_id_;
  bool follower_enable_{false};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cargo_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr follower_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr platoon_mode_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mp_control_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cargo_events_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cargo_current_id_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeaderTaskManager>());
  rclcpp::shutdown();
  return 0;
}
