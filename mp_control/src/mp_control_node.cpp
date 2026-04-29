#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace mp_control
{
namespace
{
template<typename T>
T clampValue(T value, T min_value, T max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

double vectorNorm(double x, double y, double z)
{
  return std::sqrt(x * x + y * y + z * z);
}
}  // namespace

class MpControlNode final : public rclcpp::Node
{
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GripperGoalHandle = rclcpp_action::ClientGoalHandle<GripperCommand>;

  MpControlNode()
  : Node("mp_control_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    readParameters();

    const auto sensor_qos = rclcpp::SensorDataQoS();
    const auto default_qos = rclcpp::QoS(rclcpp::KeepLast(10));

    bbox_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      bbox_topic_, default_qos,
      [this](const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) { onBbox(msg); });
    init_bbox_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      fallback_bbox_topic_, default_qos,
      [this](const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) { onBbox(msg); });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, sensor_qos,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) { onDepth(msg); });
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, sensor_qos,
      [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { onCameraInfo(msg); });
    start_sub_ = create_subscription<std_msgs::msg::Bool>(
      start_topic_, default_qos,
      [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (msg->data) {
          startSequence();
        }
      });
    cancel_sub_ = create_subscription<std_msgs::msg::Bool>(
      cancel_topic_, default_qos,
      [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (msg->data) {
          cancelSequence("cancel requested");
        }
      });

    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic_, default_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, default_qos);
    gripper_client_ = rclcpp_action::create_client<GripperCommand>(this, gripper_action_name_);

    if (auto_start_) {
      startSequence();
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, command_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      [this]() { update(); });

    publishStatus("ready; publish std_msgs/Bool true to " + start_topic_ + " to start grasping", true);
  }

private:
  struct Bbox
  {
    double x{0.0};
    double y{0.0};
    double width{0.0};
    double height{0.0};
    rclcpp::Time stamp;
  };

  struct CameraInfo
  {
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    std::uint32_t width{0};
    std::uint32_t height{0};
    std::string frame_id;
  };

  void readParameters()
  {
    bbox_topic_ = declare_parameter<std::string>("bbox_topic", "/target/tracked_bbox");
    fallback_bbox_topic_ = declare_parameter<std::string>("fallback_bbox_topic", "/target/init_bbox");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");
    twist_topic_ = declare_parameter<std::string>("twist_topic", "/servo_node/delta_twist_cmds");
    start_topic_ = declare_parameter<std::string>("start_topic", "/mp_control/start");
    cancel_topic_ = declare_parameter<std::string>("cancel_topic", "/mp_control/cancel");
    status_topic_ = declare_parameter<std::string>("status_topic", "/mp_control/status");
    gripper_action_name_ = declare_parameter<std::string>("gripper_action_name", "/gripper_controller/gripper_cmd");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    end_effector_frame_ = declare_parameter<std::string>("end_effector_frame", "end_effector_link");
    camera_frame_override_ = declare_parameter<std::string>("camera_frame_override", "");

    auto_start_ = declare_parameter<bool>("auto_start", false);
    open_gripper_on_start_ = declare_parameter<bool>("open_gripper_on_start", true);
    close_gripper_on_arrival_ = declare_parameter<bool>("close_gripper_on_arrival", true);
    command_rate_hz_ = declare_parameter<double>("command_rate_hz", 20.0);
    max_target_age_s_ = declare_parameter<double>("max_target_age_s", 0.6);
    linear_gain_ = declare_parameter<double>("linear_gain", 0.9);
    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.025);
    position_tolerance_m_ = declare_parameter<double>("position_tolerance_m", 0.035);
    close_after_stable_cycles_ = declare_parameter<int>("close_after_stable_cycles", 8);
    depth_roi_radius_px_ = declare_parameter<int>("depth_roi_radius_px", 5);
    depth_unit_scale_ = declare_parameter<double>("depth_unit_scale", 0.001);
    min_valid_depth_m_ = declare_parameter<double>("min_valid_depth_m", 0.12);
    max_valid_depth_m_ = declare_parameter<double>("max_valid_depth_m", 1.2);
    grasp_offset_x_ = declare_parameter<double>("grasp_offset_x", 0.0);
    grasp_offset_y_ = declare_parameter<double>("grasp_offset_y", 0.0);
    grasp_offset_z_ = declare_parameter<double>("grasp_offset_z", 0.0);
    gripper_open_position_ = declare_parameter<double>("gripper_open_position", 0.025);
    gripper_close_position_ = declare_parameter<double>("gripper_close_position", -0.015);
    gripper_max_effort_ = declare_parameter<double>("gripper_max_effort", -1.0);

    command_rate_hz_ = std::max(1.0, command_rate_hz_);
    max_linear_speed_ = std::max(0.0, max_linear_speed_);
    position_tolerance_m_ = std::max(0.005, position_tolerance_m_);
    close_after_stable_cycles_ = std::max(1, close_after_stable_cycles_);
    depth_roi_radius_px_ = std::max(0, depth_roi_radius_px_);
  }

  void onBbox(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg)
  {
    if (msg->data.size() < 4) {
      return;
    }
    const double width = msg->data[2];
    const double height = msg->data[3];
    if (width <= 1.0 || height <= 1.0) {
      return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_bbox_ = Bbox{
      static_cast<double>(msg->data[0]),
      static_cast<double>(msg->data[1]),
      width,
      height,
      now()};
  }

  void onDepth(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_depth_ = msg;
  }

  void onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    CameraInfo info;
    info.fx = msg->k[0];
    info.fy = msg->k[4];
    info.cx = msg->k[2];
    info.cy = msg->k[5];
    info.width = msg->width;
    info.height = msg->height;
    info.frame_id = msg->header.frame_id;
    if (info.fx <= 1.0 || info.fy <= 1.0) {
      return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_camera_info_ = info;
  }

  void startSequence()
  {
    active_ = true;
    done_ = false;
    close_sent_ = false;
    open_sent_ = false;
    stable_cycles_ = 0;
    if (open_gripper_on_start_) {
      sendGripper(gripper_open_position_);
      open_sent_ = true;
    }
    publishStatus("grasp sequence started", true);
  }

  void cancelSequence(const std::string & reason)
  {
    active_ = false;
    done_ = false;
    stable_cycles_ = 0;
    publishStop();
    publishStatus(reason, true);
  }

  void update()
  {
    if (!active_ || done_) {
      return;
    }

    const auto maybe_object = estimateObjectPoint();
    if (!maybe_object) {
      publishStop();
      publishStatus("waiting for bbox, depth, camera info, or TF");
      return;
    }

    geometry_msgs::msg::TransformStamped eef_tf;
    try {
      eef_tf = tf_buffer_.lookupTransform(target_frame_, end_effector_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      publishStop();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "end-effector TF unavailable: %s", ex.what());
      return;
    }

    const auto & object = *maybe_object;
    const double goal_x = object.point.x + grasp_offset_x_;
    const double goal_y = object.point.y + grasp_offset_y_;
    const double goal_z = object.point.z + grasp_offset_z_;

    const double eef_x = eef_tf.transform.translation.x;
    const double eef_y = eef_tf.transform.translation.y;
    const double eef_z = eef_tf.transform.translation.z;

    const double err_x = goal_x - eef_x;
    const double err_y = goal_y - eef_y;
    const double err_z = goal_z - eef_z;
    const double err_norm = vectorNorm(err_x, err_y, err_z);

    if (err_norm <= position_tolerance_m_) {
      stable_cycles_ += 1;
      publishStop();
      if (stable_cycles_ >= close_after_stable_cycles_) {
        if (close_gripper_on_arrival_ && !close_sent_) {
          sendGripper(gripper_close_position_);
          close_sent_ = true;
        }
        done_ = true;
        active_ = false;
        publishStatus("grasp target reached; gripper close command sent", true);
      } else {
        publishStatus("holding near target before closing");
      }
      return;
    }

    stable_cycles_ = 0;
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = target_frame_;
    cmd.twist.linear.x = clampValue(linear_gain_ * err_x, -max_linear_speed_, max_linear_speed_);
    cmd.twist.linear.y = clampValue(linear_gain_ * err_y, -max_linear_speed_, max_linear_speed_);
    cmd.twist.linear.z = clampValue(linear_gain_ * err_z, -max_linear_speed_, max_linear_speed_);
    twist_pub_->publish(cmd);

    std::ostringstream status;
    status << "approach err=(" << err_x << ", " << err_y << ", " << err_z
           << ") norm=" << err_norm;
    publishStatus(status.str());
  }

  std::optional<geometry_msgs::msg::PointStamped> estimateObjectPoint()
  {
    Bbox bbox;
    CameraInfo info;
    sensor_msgs::msg::Image::ConstSharedPtr depth;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!latest_bbox_ || !latest_camera_info_ || !latest_depth_) {
        return std::nullopt;
      }
      bbox = *latest_bbox_;
      info = *latest_camera_info_;
      depth = latest_depth_;
    }

    if ((now() - bbox.stamp).seconds() > max_target_age_s_) {
      return std::nullopt;
    }

    const double u = bbox.x + 0.5 * bbox.width;
    const double v = bbox.y + 0.5 * bbox.height;
    const auto depth_m = medianDepthAt(*depth, info, u, v);
    if (!depth_m) {
      return std::nullopt;
    }

    geometry_msgs::msg::PointStamped object_camera;
    object_camera.header.stamp = depth->header.stamp;
    object_camera.header.frame_id = camera_frame_override_.empty() ? info.frame_id : camera_frame_override_;
    object_camera.point.z = *depth_m;
    object_camera.point.x = (u - info.cx) * (*depth_m) / info.fx;
    object_camera.point.y = (v - info.cy) * (*depth_m) / info.fy;

    try {
      return tf_buffer_.transform(object_camera, target_frame_);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "object TF transform failed: %s", ex.what());
      return std::nullopt;
    }
  }

  std::optional<double> medianDepthAt(
    const sensor_msgs::msg::Image & depth,
    const CameraInfo & info,
    double color_u,
    double color_v) const
  {
    const double image_width = info.width > 0 ? static_cast<double>(info.width) : static_cast<double>(depth.width);
    const double image_height = info.height > 0 ? static_cast<double>(info.height) : static_cast<double>(depth.height);
    const int center_col = static_cast<int>(std::lround(color_u * static_cast<double>(depth.width) / std::max(1.0, image_width)));
    const int center_row = static_cast<int>(std::lround(color_v * static_cast<double>(depth.height) / std::max(1.0, image_height)));

    std::vector<double> samples;
    const int row_begin = clampValue(center_row - depth_roi_radius_px_, 0, static_cast<int>(depth.height) - 1);
    const int row_end = clampValue(center_row + depth_roi_radius_px_, 0, static_cast<int>(depth.height) - 1);
    const int col_begin = clampValue(center_col - depth_roi_radius_px_, 0, static_cast<int>(depth.width) - 1);
    const int col_end = clampValue(center_col + depth_roi_radius_px_, 0, static_cast<int>(depth.width) - 1);

    for (int row = row_begin; row <= row_end; ++row) {
      for (int col = col_begin; col <= col_end; ++col) {
        const auto meters = depthPixelMeters(depth, row, col);
        if (meters && *meters >= min_valid_depth_m_ && *meters <= max_valid_depth_m_) {
          samples.push_back(*meters);
        }
      }
    }

    if (samples.empty()) {
      return std::nullopt;
    }
    std::sort(samples.begin(), samples.end());
    return samples[samples.size() / 2];
  }

  std::optional<double> depthPixelMeters(const sensor_msgs::msg::Image & depth, int row, int col) const
  {
    if (row < 0 || col < 0 || row >= static_cast<int>(depth.height) || col >= static_cast<int>(depth.width)) {
      return std::nullopt;
    }

    if (depth.encoding == sensor_msgs::image_encodings::TYPE_16UC1 || depth.encoding == "16UC1") {
      const auto offset = static_cast<size_t>(row) * depth.step + static_cast<size_t>(col) * sizeof(std::uint16_t);
      if (offset + sizeof(std::uint16_t) > depth.data.size()) {
        return std::nullopt;
      }
      std::uint16_t raw = 0;
      std::memcpy(&raw, depth.data.data() + offset, sizeof(raw));
      if (raw == 0) {
        return std::nullopt;
      }
      return static_cast<double>(raw) * depth_unit_scale_;
    }

    if (depth.encoding == sensor_msgs::image_encodings::TYPE_32FC1 || depth.encoding == "32FC1") {
      const auto offset = static_cast<size_t>(row) * depth.step + static_cast<size_t>(col) * sizeof(float);
      if (offset + sizeof(float) > depth.data.size()) {
        return std::nullopt;
      }
      float raw = 0.0F;
      std::memcpy(&raw, depth.data.data() + offset, sizeof(raw));
      if (!std::isfinite(raw) || raw <= 0.0F) {
        return std::nullopt;
      }
      return static_cast<double>(raw);
    }

    return std::nullopt;
  }

  void sendGripper(double position)
  {
    if (!gripper_client_->wait_for_action_server(std::chrono::milliseconds(100))) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "gripper action server unavailable: %s", gripper_action_name_.c_str());
      return;
    }

    GripperCommand::Goal goal;
    goal.command.position = position;
    goal.command.max_effort = gripper_max_effort_;

    auto options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
    options.result_callback = [this](const GripperGoalHandle::WrappedResult & result) {
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_WARN(get_logger(), "gripper action finished with non-success result");
      }
    };
    gripper_client_->async_send_goal(goal, options);
  }

  void publishStop()
  {
    geometry_msgs::msg::TwistStamped stop;
    stop.header.stamp = now();
    stop.header.frame_id = target_frame_;
    twist_pub_->publish(stop);
  }

  void publishStatus(const std::string & text, bool force = false)
  {
    const auto stamp = now();
    if (!force && last_status_stamp_.nanoseconds() != 0 &&
        (stamp - last_status_stamp_).seconds() < 0.5) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
    last_status_stamp_ = stamp;
  }

  std::string bbox_topic_;
  std::string fallback_bbox_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string twist_topic_;
  std::string start_topic_;
  std::string cancel_topic_;
  std::string status_topic_;
  std::string gripper_action_name_;
  std::string target_frame_;
  std::string end_effector_frame_;
  std::string camera_frame_override_;

  bool auto_start_{false};
  bool open_gripper_on_start_{true};
  bool close_gripper_on_arrival_{true};
  double command_rate_hz_{20.0};
  double max_target_age_s_{0.6};
  double linear_gain_{0.9};
  double max_linear_speed_{0.025};
  double position_tolerance_m_{0.035};
  int close_after_stable_cycles_{8};
  int depth_roi_radius_px_{5};
  double depth_unit_scale_{0.001};
  double min_valid_depth_m_{0.12};
  double max_valid_depth_m_{1.2};
  double grasp_offset_x_{0.0};
  double grasp_offset_y_{0.0};
  double grasp_offset_z_{0.0};
  double gripper_open_position_{0.025};
  double gripper_close_position_{-0.015};
  double gripper_max_effort_{-1.0};

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr bbox_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr init_bbox_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex data_mutex_;
  std::optional<Bbox> latest_bbox_;
  std::optional<CameraInfo> latest_camera_info_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;

  bool active_{false};
  bool done_{false};
  bool open_sent_{false};
  bool close_sent_{false};
  int stable_cycles_{0};
  rclcpp::Time last_status_stamp_;
};

}  // namespace mp_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mp_control::MpControlNode>());
  rclcpp::shutdown();
  return 0;
}
