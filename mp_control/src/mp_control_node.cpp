#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <future>
#include <iomanip>
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
#include <std_srvs/srv/trigger.hpp>
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
  using Trigger = std_srvs::srv::Trigger;

  MpControlNode()
  : Node("mp_control_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    readParameters();

    const auto sensor_qos = rclcpp::SensorDataQoS();
    const auto default_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    const auto current_id_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    bbox_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      bbox_topic_, default_qos,
      [this](const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) { onBbox(msg); });
    init_bbox_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      fallback_bbox_topic_, default_qos,
      [this](const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) { onBbox(msg); });
    eef_bbox_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      eef_bbox_topic_, default_qos,
      [this](const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) { onEefBbox(msg); });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, sensor_qos,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) { onDepth(msg); });
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, sensor_qos,
      [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { onCameraInfo(msg); });
    eef_camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      eef_camera_info_topic_, sensor_qos,
      [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { onEefCameraInfo(msg); });
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
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, status_qos);
    cargo_event_pub_ = create_publisher<std_msgs::msg::String>(cargo_event_topic_, default_qos);
    cargo_current_id_pub_ =
      create_publisher<std_msgs::msg::String>(cargo_current_id_topic_, current_id_qos);
    gripper_client_ = rclcpp_action::create_client<GripperCommand>(this, gripper_action_name_);
    servo_start_client_ = create_client<Trigger>("/servo_node/start_servo");

    if (auto_start_) {
      startSequence();
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, command_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      [this]() { update(); });

    RCLCPP_INFO(
      get_logger(),
      "mp_control started: bbox=%s depth=%s camera_info=%s eef_bbox=%s eef_camera_info=%s twist=%s target_frame=%s eef_frame=%s auto_start_on_bbox=%s eef_refinement=%s",
      bbox_topic_.c_str(), depth_topic_.c_str(), camera_info_topic_.c_str(),
      eef_bbox_topic_.c_str(), eef_camera_info_topic_.c_str(), twist_topic_.c_str(),
      target_frame_.c_str(), end_effector_frame_.c_str(),
      auto_start_on_bbox_ ? "true" : "false",
      use_eef_refinement_ ? "true" : "false");

    publishStatus("ready; publish std_msgs/Bool true to " + start_topic_ + " to start grasping", true);
  }

private:
  enum class GraspStage
  {
    DEPTH_APPROACH,
    EEF_REFINE
  };

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
    eef_bbox_topic_ = declare_parameter<std::string>("eef_bbox_topic", "/target/eef_tracked_bbox");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");
    eef_camera_info_topic_ = declare_parameter<std::string>("eef_camera_info_topic", "/eef_camera/camera_info");
    twist_topic_ = declare_parameter<std::string>("twist_topic", "/servo_node/delta_twist_cmds");
    start_topic_ = declare_parameter<std::string>("start_topic", "/mp_control/start");
    cancel_topic_ = declare_parameter<std::string>("cancel_topic", "/mp_control/cancel");
    status_topic_ = declare_parameter<std::string>("status_topic", "/mp_control/status");
    cargo_event_topic_ = declare_parameter<std::string>("cargo_event_topic", "/cargo/events");
    cargo_current_id_topic_ =
      declare_parameter<std::string>("cargo_current_id_topic", "/cargo/current_id");
    cargo_id_prefix_ = declare_parameter<std::string>("cargo_id_prefix", "PKG");
    cargo_sequence_next_ = declare_parameter<int>("cargo_sequence_start", 1);
    gripper_action_name_ = declare_parameter<std::string>("gripper_action_name", "/gripper_controller/gripper_cmd");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    end_effector_frame_ = declare_parameter<std::string>("end_effector_frame", "end_effector_link");
    camera_frame_override_ = declare_parameter<std::string>("camera_frame_override", "");
    eef_camera_frame_override_ =
      declare_parameter<std::string>("eef_camera_frame_override", "eef_usb_camera_optical_frame");

    auto_start_ = declare_parameter<bool>("auto_start", false);
    auto_start_on_bbox_ = declare_parameter<bool>("auto_start_on_bbox", false);
    start_servo_on_start_ = declare_parameter<bool>("start_servo_on_start", true);
    open_gripper_on_start_ = declare_parameter<bool>("open_gripper_on_start", true);
    close_gripper_on_arrival_ = declare_parameter<bool>("close_gripper_on_arrival", true);
    use_eef_refinement_ = declare_parameter<bool>("use_eef_refinement", true);
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
    eef_refinement_switch_distance_m_ = declare_parameter<double>("eef_refinement_switch_distance_m", 0.12);
    eef_final_depth_m_ = declare_parameter<double>("eef_final_depth_m", 0.08);
    eef_center_tolerance_px_ = declare_parameter<double>("eef_center_tolerance_px", 18.0);
    eef_depth_tolerance_m_ = declare_parameter<double>("eef_depth_tolerance_m", 0.018);
    eef_refine_lateral_gain_ = declare_parameter<double>("eef_refine_lateral_gain", 0.8);
    eef_refine_depth_gain_ = declare_parameter<double>("eef_refine_depth_gain", 0.5);
    eef_refine_max_linear_speed_ = declare_parameter<double>("eef_refine_max_linear_speed", 0.012);
    gripper_open_position_ = declare_parameter<double>("gripper_open_position", 0.025);
    gripper_close_position_ = declare_parameter<double>("gripper_close_position", -0.015);
    gripper_max_effort_ = declare_parameter<double>("gripper_max_effort", -1.0);

    command_rate_hz_ = std::max(1.0, command_rate_hz_);
    max_linear_speed_ = std::max(0.0, max_linear_speed_);
    position_tolerance_m_ = std::max(0.005, position_tolerance_m_);
    close_after_stable_cycles_ = std::max(1, close_after_stable_cycles_);
    depth_roi_radius_px_ = std::max(0, depth_roi_radius_px_);
    eef_refinement_switch_distance_m_ = std::max(0.01, eef_refinement_switch_distance_m_);
    eef_final_depth_m_ = std::max(0.0, eef_final_depth_m_);
    eef_center_tolerance_px_ = std::max(1.0, eef_center_tolerance_px_);
    eef_depth_tolerance_m_ = std::max(0.001, eef_depth_tolerance_m_);
    eef_refine_max_linear_speed_ = std::max(0.0, eef_refine_max_linear_speed_);
    cargo_sequence_next_ = std::max(1, cargo_sequence_next_);
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

    bool should_auto_start = false;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_bbox_ = Bbox{
        static_cast<double>(msg->data[0]),
        static_cast<double>(msg->data[1]),
        width,
        height,
        now()};
      should_auto_start = auto_start_on_bbox_ && !active_ && !done_;
    }

    if (should_auto_start) {
      startSequence();
    }
  }

  void onDepth(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_depth_ = msg;
  }

  void onEefBbox(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg)
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
    latest_eef_bbox_ = Bbox{
      static_cast<double>(msg->data[0]),
      static_cast<double>(msg->data[1]),
      width,
      height,
      now()};
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

  void onEefCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
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
    latest_eef_camera_info_ = info;
  }

  void startSequence()
  {
    active_ = true;
    done_ = false;
    close_sent_ = false;
    open_sent_ = false;
    stable_cycles_ = 0;
    stage_ = GraspStage::DEPTH_APPROACH;
    assignCargoId();
    publishCargoEvent("assigned", true);
    if (open_gripper_on_start_) {
      sendGripper(gripper_open_position_);
      open_sent_ = true;
    }
    if (start_servo_on_start_) {
      startMoveItServo();
    }
    publishStatus("grasp sequence started", true);
  }

  void cancelSequence(const std::string & reason)
  {
    active_ = false;
    done_ = false;
    stable_cycles_ = 0;
    stage_ = GraspStage::DEPTH_APPROACH;
    publishStop();
    publishCargoEvent("cancelled", true);
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

    if (stage_ == GraspStage::EEF_REFINE) {
      updateEefRefinement(object);
      return;
    }

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

    if (use_eef_refinement_ && err_norm <= eef_refinement_switch_distance_m_) {
      stage_ = GraspStage::EEF_REFINE;
      stable_cycles_ = 0;
      publishStatus("switching to end-effector camera refinement", true);
      updateEefRefinement(object);
      return;
    }

    if (err_norm <= position_tolerance_m_) {
      stable_cycles_ += 1;
      publishStop();
      if (stable_cycles_ >= close_after_stable_cycles_) {
        if (close_gripper_on_arrival_ && !close_sent_) {
          sendGripper(gripper_close_position_);
          close_sent_ = true;
          publishCargoEvent("picked", true);
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

  void updateEefRefinement(const geometry_msgs::msg::PointStamped & object_in_target)
  {
    Bbox bbox;
    CameraInfo info;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!latest_eef_bbox_ || !latest_eef_camera_info_) {
        publishStop();
        publishStatus("waiting for end-effector camera bbox or camera info");
        return;
      }
      bbox = *latest_eef_bbox_;
      info = *latest_eef_camera_info_;
    }

    if ((now() - bbox.stamp).seconds() > max_target_age_s_) {
      publishStop();
      publishStatus("waiting for fresh end-effector camera bbox");
      return;
    }

    const std::string eef_camera_frame =
      eef_camera_frame_override_.empty() ? info.frame_id : eef_camera_frame_override_;

    geometry_msgs::msg::PointStamped object_latest = object_in_target;
    object_latest.header.stamp = builtin_interfaces::msg::Time();

    geometry_msgs::msg::PointStamped object_in_eef_camera;
    try {
      object_in_eef_camera = tf_buffer_.transform(object_latest, eef_camera_frame);
    } catch (const tf2::TransformException & ex) {
      publishStop();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "end-effector camera refinement TF unavailable: %s", ex.what());
      return;
    }

    const double u = bbox.x + 0.5 * bbox.width;
    const double v = bbox.y + 0.5 * bbox.height;
    const double err_u_px = u - info.cx;
    const double err_v_px = v - info.cy;
    const double z_m = std::max(eef_final_depth_m_, object_in_eef_camera.point.z);
    const double lateral_m = err_u_px * z_m / info.fx;
    const double vertical_m = err_v_px * z_m / info.fy;
    const double depth_error_m = object_in_eef_camera.point.z - eef_final_depth_m_;

    const bool centered =
      std::abs(err_u_px) <= eef_center_tolerance_px_ &&
      std::abs(err_v_px) <= eef_center_tolerance_px_;
    const bool at_depth = std::abs(depth_error_m) <= eef_depth_tolerance_m_;

    if (centered && at_depth) {
      stable_cycles_ += 1;
      publishStop();
      if (stable_cycles_ >= close_after_stable_cycles_) {
        if (close_gripper_on_arrival_ && !close_sent_) {
          sendGripper(gripper_close_position_);
          close_sent_ = true;
          publishCargoEvent("picked", true);
        }
        done_ = true;
        active_ = false;
        publishStatus("eef camera refined grasp reached; gripper close command sent", true);
      } else {
        publishStatus("eef camera aligned; holding before closing");
      }
      return;
    }

    stable_cycles_ = 0;
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = eef_camera_frame;
    cmd.twist.linear.x = clampValue(
      eef_refine_lateral_gain_ * lateral_m,
      -eef_refine_max_linear_speed_, eef_refine_max_linear_speed_);
    cmd.twist.linear.y = clampValue(
      eef_refine_lateral_gain_ * vertical_m,
      -eef_refine_max_linear_speed_, eef_refine_max_linear_speed_);
    cmd.twist.linear.z = clampValue(
      eef_refine_depth_gain_ * depth_error_m,
      -eef_refine_max_linear_speed_, eef_refine_max_linear_speed_);
    twist_pub_->publish(cmd);

    std::ostringstream status;
    status << "eef refine pixel_err=(" << err_u_px << ", " << err_v_px
           << ") depth_err=" << depth_error_m
           << " cmd_frame=" << eef_camera_frame;
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

  void startMoveItServo()
  {
    if (!servo_start_client_->wait_for_service(std::chrono::milliseconds(100))) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "MoveIt Servo start service unavailable: /servo_node/start_servo");
      return;
    }

    auto future = servo_start_client_->async_send_request(std::make_shared<Trigger::Request>());
    const auto result = future.wait_for(std::chrono::seconds(1));
    if (result != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "MoveIt Servo start service call timed out");
      return;
    }

    const auto response = future.get();
    if (!response->success) {
      RCLCPP_WARN(get_logger(), "MoveIt Servo start request returned false: %s", response->message.c_str());
    }
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

  void assignCargoId()
  {
    std::ostringstream id;
    id << cargo_id_prefix_ << "-" << std::setw(6) << std::setfill('0') << cargo_sequence_next_++;
    current_cargo_id_ = id.str();
    std_msgs::msg::String msg;
    msg.data = current_cargo_id_;
    cargo_current_id_pub_->publish(msg);
  }

  void publishCargoEvent(const std::string & event, bool force_current = false)
  {
    if (current_cargo_id_.empty() || force_current) {
      std_msgs::msg::String current;
      current.data = current_cargo_id_;
      cargo_current_id_pub_->publish(current);
    }

    const auto stamp = now();
    const auto stamp_ns = stamp.nanoseconds();
    std_msgs::msg::String msg;
    std::ostringstream event_json;
    event_json << "{\"cargo_id\":\"" << current_cargo_id_
               << "\",\"event\":\"" << event
               << "\",\"stamp\":{\"sec\":" << stamp_ns / 1000000000LL
               << ",\"nanosec\":" << stamp_ns % 1000000000LL
               << "}}";
    msg.data = event_json.str();
    cargo_event_pub_->publish(msg);
  }

  std::string bbox_topic_;
  std::string fallback_bbox_topic_;
  std::string eef_bbox_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string eef_camera_info_topic_;
  std::string twist_topic_;
  std::string start_topic_;
  std::string cancel_topic_;
  std::string status_topic_;
  std::string cargo_event_topic_;
  std::string cargo_current_id_topic_;
  std::string cargo_id_prefix_;
  std::string gripper_action_name_;
  std::string target_frame_;
  std::string end_effector_frame_;
  std::string camera_frame_override_;
  std::string eef_camera_frame_override_;

  bool auto_start_{false};
  bool auto_start_on_bbox_{false};
  bool start_servo_on_start_{true};
  bool open_gripper_on_start_{true};
  bool close_gripper_on_arrival_{true};
  bool use_eef_refinement_{true};
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
  double eef_refinement_switch_distance_m_{0.12};
  double eef_final_depth_m_{0.08};
  double eef_center_tolerance_px_{18.0};
  double eef_depth_tolerance_m_{0.018};
  double eef_refine_lateral_gain_{0.8};
  double eef_refine_depth_gain_{0.5};
  double eef_refine_max_linear_speed_{0.012};
  double gripper_open_position_{0.025};
  double gripper_close_position_{-0.015};
  double gripper_max_effort_{-1.0};
  int cargo_sequence_next_{1};
  std::string current_cargo_id_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr bbox_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr init_bbox_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr eef_bbox_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr eef_camera_info_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cargo_event_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cargo_current_id_pub_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
  rclcpp::Client<Trigger>::SharedPtr servo_start_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex data_mutex_;
  std::optional<Bbox> latest_bbox_;
  std::optional<Bbox> latest_eef_bbox_;
  std::optional<CameraInfo> latest_camera_info_;
  std::optional<CameraInfo> latest_eef_camera_info_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;

  bool active_{false};
  bool done_{false};
  bool open_sent_{false};
  bool close_sent_{false};
  GraspStage stage_{GraspStage::DEPTH_APPROACH};
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
