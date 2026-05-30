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
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
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
    const auto init_bbox_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    const auto current_id_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    bbox_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      bbox_topic_, default_qos,
      [this](const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) { onBbox(msg); });
    if (use_fallback_bbox_for_control_ && !fallback_bbox_topic_.empty()) {
      init_bbox_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        fallback_bbox_topic_, init_bbox_qos,
        [this](const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) { onBbox(msg); });
    }
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
    eef_init_bbox_pub_ =
      create_publisher<std_msgs::msg::Float32MultiArray>(eef_init_bbox_topic_, init_bbox_qos);
    eef_auto_init_enable_pub_ =
      create_publisher<std_msgs::msg::Bool>(eef_auto_init_enable_topic_, init_bbox_qos);
    base_hold_pub_ =
      create_publisher<std_msgs::msg::Bool>(base_hold_topic_, init_bbox_qos);
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
    TRIANGULATION_EXTEND,
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

  struct GripperTarget
  {
    double position{0.0};
    double object_width_m{0.0};
    bool measured{false};
  };

  struct Ray
  {
    tf2::Vector3 origin;
    tf2::Vector3 direction;
  };

  void readParameters()
  {
    bbox_topic_ = declare_parameter<std::string>("bbox_topic", "/target/tracked_bbox");
    fallback_bbox_topic_ = declare_parameter<std::string>("fallback_bbox_topic", "/target/init_bbox");
    eef_bbox_topic_ = declare_parameter<std::string>("eef_bbox_topic", "/target/eef_tracked_bbox");
    eef_init_bbox_topic_ = declare_parameter<std::string>("eef_init_bbox_topic", "/target/eef_init_bbox");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");
    eef_camera_info_topic_ = declare_parameter<std::string>("eef_camera_info_topic", "/eef_camera/camera_info");
    eef_auto_init_enable_topic_ =
      declare_parameter<std::string>("eef_auto_init_enable_topic", "/target/eef_auto_init_enable");
    base_hold_topic_ = declare_parameter<std::string>("base_hold_topic", "/target/base_hold");
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
    allow_eef_camera_info_fallback_ =
      declare_parameter<bool>("allow_eef_camera_info_fallback", true);
    eef_camera_fallback_width_px_ =
      declare_parameter<int>("eef_camera_fallback_width_px", 640);
    eef_camera_fallback_height_px_ =
      declare_parameter<int>("eef_camera_fallback_height_px", 480);
    eef_camera_fallback_fx_ = declare_parameter<double>("eef_camera_fallback_fx", 554.0);
    eef_camera_fallback_fy_ = declare_parameter<double>("eef_camera_fallback_fy", 554.0);

    auto_start_ = declare_parameter<bool>("auto_start", false);
    auto_start_on_bbox_ = declare_parameter<bool>("auto_start_on_bbox", false);
    use_fallback_bbox_for_control_ =
      declare_parameter<bool>("use_fallback_bbox_for_control", false);
    start_servo_on_start_ = declare_parameter<bool>("start_servo_on_start", true);
    open_gripper_on_start_ = declare_parameter<bool>("open_gripper_on_start", true);
    close_gripper_on_arrival_ = declare_parameter<bool>("close_gripper_on_arrival", true);
    use_eef_refinement_ = declare_parameter<bool>("use_eef_refinement", true);
    wait_for_base_approach_ = declare_parameter<bool>("wait_for_base_approach", false);
    auto_init_eef_tracker_from_object_ =
      declare_parameter<bool>("auto_init_eef_tracker_from_object", true);
    use_depthless_triangulation_ =
      declare_parameter<bool>("use_depthless_triangulation", false);
    use_color_triangulation_after_min_depth_ =
      declare_parameter<bool>("use_color_triangulation_after_min_depth", false);
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
    eef_refinement_start_depth_m_ =
      declare_parameter<double>("eef_refinement_start_depth_m", min_valid_depth_m_);
    eef_yolo_pre_enable_depth_m_ =
      declare_parameter<double>("eef_yolo_pre_enable_depth_m", eef_refinement_start_depth_m_);
    min_depth_handoff_margin_m_ =
      declare_parameter<double>("min_depth_handoff_margin_m", 0.02);
    min_depth_handoff_bbox_area_ratio_ =
      declare_parameter<double>("min_depth_handoff_bbox_area_ratio", 0.30);
    min_depth_handoff_bbox_height_ratio_ =
      declare_parameter<double>("min_depth_handoff_bbox_height_ratio", 0.75);
    eef_refinement_start_object_x_m_ =
      declare_parameter<double>("eef_refinement_start_object_x_m", 0.50);
    arm_start_max_error_m_ = declare_parameter<double>("arm_start_max_error_m", 0.40);
    arm_start_max_object_x_m_ = declare_parameter<double>("arm_start_max_object_x_m", 0.60);
    object_height_m_ = declare_parameter<double>("object_height_m", 0.10);
    eef_init_bbox_min_size_px_ = declare_parameter<double>("eef_init_bbox_min_size_px", 32.0);
    eef_init_bbox_max_size_px_ = declare_parameter<double>("eef_init_bbox_max_size_px", 180.0);
    eef_init_bbox_padding_scale_ = declare_parameter<double>("eef_init_bbox_padding_scale", 1.6);
    eef_init_bbox_republish_period_s_ =
      declare_parameter<double>("eef_init_bbox_republish_period_s", 0.5);
    eef_final_depth_m_ = declare_parameter<double>("eef_final_depth_m", 0.08);
    object_pregrasp_standoff_m_ = declare_parameter<double>("object_pregrasp_standoff_m", 0.08);
    object_pregrasp_min_z_m_ = declare_parameter<double>("object_pregrasp_min_z_m", 0.50);
    object_pregrasp_lower_standoff_m_ =
      declare_parameter<double>("object_pregrasp_lower_standoff_m", 0.02);
    object_pregrasp_min_lower_z_m_ =
      declare_parameter<double>("object_pregrasp_min_lower_z_m", 0.12);
    object_pregrasp_enable_lowering_ =
      declare_parameter<bool>("object_pregrasp_enable_lowering", false);
    eef_center_tolerance_px_ = declare_parameter<double>("eef_center_tolerance_px", 18.0);
    eef_close_tolerance_px_ = declare_parameter<double>("eef_close_tolerance_px", eef_center_tolerance_px_);
    eef_depth_tolerance_m_ = declare_parameter<double>("eef_depth_tolerance_m", 0.018);
    eef_refine_lateral_gain_ = declare_parameter<double>("eef_refine_lateral_gain", 0.8);
    eef_refine_depth_gain_ = declare_parameter<double>("eef_refine_depth_gain", 0.5);
    eef_refine_max_linear_speed_ = declare_parameter<double>("eef_refine_max_linear_speed", 0.012);
    triangulation_extend_x_m_ = declare_parameter<double>("triangulation_extend_x_m", 0.25);
    triangulation_extend_y_m_ = declare_parameter<double>("triangulation_extend_y_m", 0.0);
    triangulation_extend_z_m_ = declare_parameter<double>("triangulation_extend_z_m", 0.12);
    triangulation_extend_tolerance_m_ =
      declare_parameter<double>("triangulation_extend_tolerance_m", 0.025);
    triangulation_extend_gain_ = declare_parameter<double>("triangulation_extend_gain", 0.7);
    triangulation_extend_max_speed_ =
      declare_parameter<double>("triangulation_extend_max_speed", 0.015);
    triangulation_min_range_m_ = declare_parameter<double>("triangulation_min_range_m", 0.06);
    triangulation_max_range_m_ = declare_parameter<double>("triangulation_max_range_m", 1.0);
    triangulation_max_ray_gap_m_ = declare_parameter<double>("triangulation_max_ray_gap_m", 0.08);
    use_eef_front_camera_extrinsic_override_ =
      declare_parameter<bool>("use_eef_front_camera_extrinsic_override", false);
    eef_front_camera_offset_x_m_ =
      declare_parameter<double>("eef_front_camera_offset_x_m", 0.0);
    eef_front_camera_offset_y_m_ =
      declare_parameter<double>("eef_front_camera_offset_y_m", 0.0);
    eef_front_camera_offset_z_m_ =
      declare_parameter<double>("eef_front_camera_offset_z_m", 0.0);
    color_triangulation_base_stop_object_x_m_ =
      declare_parameter<double>("color_triangulation_base_stop_object_x_m", 0.30);
    color_triangulation_min_object_x_m_ =
      declare_parameter<double>("color_triangulation_min_object_x_m", 0.05);
    gripper_open_position_ = declare_parameter<double>("gripper_open_position", 0.025);
    gripper_close_position_ = declare_parameter<double>("gripper_close_position", -0.015);
    gripper_max_effort_ = declare_parameter<double>("gripper_max_effort", -1.0);
    gripper_width_control_enabled_ = declare_parameter<bool>("gripper_width_control_enabled", true);
    gripper_fallback_object_width_m_ =
      declare_parameter<double>("gripper_fallback_object_width_m", 0.06);
    gripper_finger_home_half_gap_m_ =
      declare_parameter<double>("gripper_finger_home_half_gap_m", 0.021);
    gripper_pre_grasp_clearance_m_ =
      declare_parameter<double>("gripper_pre_grasp_clearance_m", 0.012);
    gripper_grasp_compression_m_ =
      declare_parameter<double>("gripper_grasp_compression_m", 0.002);
    gripper_min_position_ = declare_parameter<double>("gripper_min_position", -0.010);
    gripper_max_position_ = declare_parameter<double>("gripper_max_position", 0.019);
    gripper_min_measured_object_width_m_ =
      declare_parameter<double>("gripper_min_measured_object_width_m", 0.01);
    gripper_max_measured_object_width_m_ =
      declare_parameter<double>("gripper_max_measured_object_width_m", 0.08);

    command_rate_hz_ = std::max(1.0, command_rate_hz_);
    max_linear_speed_ = std::max(0.0, max_linear_speed_);
    position_tolerance_m_ = std::max(0.005, position_tolerance_m_);
    close_after_stable_cycles_ = std::max(1, close_after_stable_cycles_);
    depth_roi_radius_px_ = std::max(0, depth_roi_radius_px_);
    eef_refinement_switch_distance_m_ = std::max(0.01, eef_refinement_switch_distance_m_);
    eef_refinement_start_depth_m_ = std::max(min_valid_depth_m_, eef_refinement_start_depth_m_);
    eef_yolo_pre_enable_depth_m_ =
      std::max(eef_refinement_start_depth_m_, eef_yolo_pre_enable_depth_m_);
    min_depth_handoff_margin_m_ = std::max(0.0, min_depth_handoff_margin_m_);
    min_depth_handoff_bbox_area_ratio_ =
      clampValue(min_depth_handoff_bbox_area_ratio_, 0.0, 1.0);
    min_depth_handoff_bbox_height_ratio_ =
      clampValue(min_depth_handoff_bbox_height_ratio_, 0.0, 1.0);
    eef_refinement_start_object_x_m_ = std::max(0.01, eef_refinement_start_object_x_m_);
    arm_start_max_error_m_ = std::max(0.05, arm_start_max_error_m_);
    arm_start_max_object_x_m_ = std::max(0.05, arm_start_max_object_x_m_);
    object_height_m_ = std::max(0.01, object_height_m_);
    eef_init_bbox_min_size_px_ = std::max(2.0, eef_init_bbox_min_size_px_);
    eef_init_bbox_max_size_px_ = std::max(eef_init_bbox_min_size_px_, eef_init_bbox_max_size_px_);
    eef_init_bbox_padding_scale_ = std::max(1.0, eef_init_bbox_padding_scale_);
    eef_init_bbox_republish_period_s_ = std::max(0.1, eef_init_bbox_republish_period_s_);
    eef_final_depth_m_ = std::max(0.0, eef_final_depth_m_);
    object_pregrasp_standoff_m_ = std::max(0.0, object_pregrasp_standoff_m_);
    object_pregrasp_min_z_m_ = std::max(0.0, object_pregrasp_min_z_m_);
    object_pregrasp_lower_standoff_m_ = std::max(0.0, object_pregrasp_lower_standoff_m_);
    object_pregrasp_min_lower_z_m_ = std::max(0.0, object_pregrasp_min_lower_z_m_);
    eef_center_tolerance_px_ = std::max(1.0, eef_center_tolerance_px_);
    eef_close_tolerance_px_ = std::max(eef_center_tolerance_px_, eef_close_tolerance_px_);
    eef_depth_tolerance_m_ = std::max(0.001, eef_depth_tolerance_m_);
    eef_refine_max_linear_speed_ = std::max(0.0, eef_refine_max_linear_speed_);
    triangulation_extend_tolerance_m_ = std::max(0.005, triangulation_extend_tolerance_m_);
    triangulation_extend_gain_ = std::max(0.0, triangulation_extend_gain_);
    triangulation_extend_max_speed_ = std::max(0.0, triangulation_extend_max_speed_);
    triangulation_min_range_m_ = std::max(0.01, triangulation_min_range_m_);
    triangulation_max_range_m_ = std::max(triangulation_min_range_m_, triangulation_max_range_m_);
    triangulation_max_ray_gap_m_ = std::max(0.005, triangulation_max_ray_gap_m_);
    if (!std::isfinite(eef_front_camera_offset_x_m_)) {
      eef_front_camera_offset_x_m_ = 0.0;
    }
    if (!std::isfinite(eef_front_camera_offset_y_m_)) {
      eef_front_camera_offset_y_m_ = 0.0;
    }
    if (!std::isfinite(eef_front_camera_offset_z_m_)) {
      eef_front_camera_offset_z_m_ = 0.0;
    }
    color_triangulation_base_stop_object_x_m_ =
      std::max(0.01, color_triangulation_base_stop_object_x_m_);
    color_triangulation_min_object_x_m_ =
      clampValue(color_triangulation_min_object_x_m_, 0.0, color_triangulation_base_stop_object_x_m_);
    cargo_sequence_next_ = std::max(1, cargo_sequence_next_);
    if (gripper_min_position_ > gripper_max_position_) {
      std::swap(gripper_min_position_, gripper_max_position_);
    }
    if (gripper_min_measured_object_width_m_ > gripper_max_measured_object_width_m_) {
      std::swap(gripper_min_measured_object_width_m_, gripper_max_measured_object_width_m_);
    }
    eef_camera_fallback_width_px_ = std::max(1, eef_camera_fallback_width_px_);
    eef_camera_fallback_height_px_ = std::max(1, eef_camera_fallback_height_px_);
    eef_camera_fallback_fx_ = std::max(1.0, eef_camera_fallback_fx_);
    eef_camera_fallback_fy_ = std::max(1.0, eef_camera_fallback_fy_);
    gripper_open_position_ = clampValue(
      gripper_open_position_, gripper_min_position_, gripper_max_position_);
    gripper_close_position_ = clampValue(
      gripper_close_position_, gripper_min_position_, gripper_max_position_);
    gripper_fallback_object_width_m_ = std::max(0.0, gripper_fallback_object_width_m_);
    gripper_finger_home_half_gap_m_ = std::max(0.0, gripper_finger_home_half_gap_m_);
    gripper_pre_grasp_clearance_m_ = std::max(0.0, gripper_pre_grasp_clearance_m_);
    gripper_grasp_compression_m_ = std::max(0.0, gripper_grasp_compression_m_);
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
    if (!eef_refinement_requested_) {
      return;
    }
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
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      eef_refinement_requested_ = false;
      latest_eef_bbox_.reset();
    }
    object_pregrasp_horizontal_done_ = false;
    min_depth_reached_ = false;
    latest_depth_object_in_target_.reset();
    latest_depth_object_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    publishEefAutoInitEnable(false);
    publishBaseHold(false);
    last_eef_init_bbox_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    stable_cycles_ = 0;
    stage_ = GraspStage::DEPTH_APPROACH;
    assignCargoId();
    publishCargoEvent("assigned", true);
    if (open_gripper_on_start_) {
      sendGripperOpenForObject();
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
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      eef_refinement_requested_ = false;
      latest_eef_bbox_.reset();
    }
    object_pregrasp_horizontal_done_ = false;
    min_depth_reached_ = false;
    latest_depth_object_in_target_.reset();
    latest_depth_object_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    publishEefAutoInitEnable(false);
    publishBaseHold(false);
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

    geometry_msgs::msg::TransformStamped eef_tf;
    try {
      eef_tf = tf_buffer_.lookupTransform(target_frame_, end_effector_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      publishStop();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "end-effector TF unavailable: %s", ex.what());
      return;
    }

    std::string object_block_reason;
    auto maybe_object = estimateObjectPoint(&object_block_reason);
    bool using_latched_depth_for_pregrasp = false;
    bool command_published = false;
    if (useColorTriangulationAfterMinDepth()) {
      std::string color_reason;
      auto front_size_object = estimateObjectPointFromFrontBboxSize();
      auto depth_reference = front_size_object ? front_size_object :
        (maybe_object ? maybe_object : latestDepthObjectInTarget());
      if (depth_reference) {
        prepareEefColorTriangulation(*depth_reference, &color_reason);
      } else {
        publishEefAutoInitEnable(true);
        color_reason = "depth limit reached; end-effector YOLO enabled";
      }

      auto maybe_color_object = triangulateObjectPoint(&color_reason, false);
      if (maybe_color_object &&
          maybe_color_object->point.x < color_triangulation_min_object_x_m_) {
        std::ostringstream reason;
        reason << "color triangulation object_x=" << maybe_color_object->point.x
               << " below " << color_triangulation_min_object_x_m_;
        color_reason = reason.str();
        maybe_color_object.reset();
      }

      if (!maybe_color_object) {
        stable_cycles_ = 0;
        publishBaseHold(false);
        publishStop();
        std::ostringstream status;
        status << "after depth limit; waiting for close-range color triangulation: "
               << color_reason
               << "; base continuing toward object_x="
               << color_triangulation_base_stop_object_x_m_;
        publishStatus(status.str());
        return;
      } else {
        maybe_object = maybe_color_object;
        object_block_reason.clear();
      }

      const double color_goal_x = maybe_object->point.x + grasp_offset_x_;
      if (!using_latched_depth_for_pregrasp &&
          color_goal_x > color_triangulation_base_stop_object_x_m_) {
        stable_cycles_ = 0;
        publishBaseHold(false);
        publishStop();
        std::ostringstream status;
        status << "color triangulation approach: object_x=" << color_goal_x
               << " target_stop_x=" << color_triangulation_base_stop_object_x_m_
               << "; depth ignored after " << eef_refinement_start_depth_m_ << "m";
        publishStatus(status.str());
        return;
      }
    }

    if (!maybe_object && use_depthless_triangulation_ &&
        !use_color_triangulation_after_min_depth_ &&
        isDepthUnavailableReason(object_block_reason)) {
      publishBaseHold(true);
      maybe_object = estimateObjectPointByTriangulation(
        eef_tf, &object_block_reason, &command_published);
    }

    if (!maybe_object) {
      if (use_eef_refinement_ &&
          (!use_color_triangulation_after_min_depth_ || useColorTriangulationAfterMinDepth()) &&
          shouldHoldForEefRefinement()) {
        publishBaseHold(true);
      }
      if (!command_published) {
        publishStop();
      }
      publishStatus("waiting for " + object_block_reason);
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

    const bool object_x_ready_for_eef = use_color_triangulation_after_min_depth_ ?
      (using_latched_depth_for_pregrasp || goal_x <= color_triangulation_base_stop_object_x_m_) :
      goal_x <= eef_refinement_start_object_x_m_;
    const bool depth_ready_for_eef = use_color_triangulation_after_min_depth_ ?
      false :
      shouldStartEefRefinementByDepth();
    const bool eef_candidate =
      using_latched_depth_for_pregrasp ||
      shouldHoldForEefRefinement() ||
      depth_ready_for_eef ||
      object_x_ready_for_eef ||
      err_norm <= eef_refinement_switch_distance_m_;
    const bool use_eef_now =
      use_eef_refinement_ &&
      (!use_color_triangulation_after_min_depth_ || object_x_ready_for_eef) &&
      eef_candidate;
    if (use_eef_now) {
      publishBaseHold(true);
      bool extension_cmd_published = false;
      if (!moveArmToObjectPregraspPose(eef_tf, object, &extension_cmd_published)) {
        return;
      }
      if (!prepareEefRefinement(object)) {
        return;
      }
      stage_ = GraspStage::EEF_REFINE;
      stable_cycles_ = 0;
      publishStatus("base stopped, depth+EEF ready; arm extended toward object and switching to refinement", true);
      updateEefRefinement(object);
      return;
    }

    const double forward_err = std::max(0.0, err_x);
    if (wait_for_base_approach_ &&
        (forward_err > arm_start_max_error_m_ || goal_x > arm_start_max_object_x_m_)) {
      stable_cycles_ = 0;
      publishBaseHold(false);
      publishStop();
      std::ostringstream status;
      status << "waiting for base approach before arm motion: goal_x=" << goal_x
             << " forward_err=" << forward_err
             << " err_norm=" << err_norm;
      publishStatus(status.str());
      return;
    }

    if (err_norm <= position_tolerance_m_) {
      stable_cycles_ += 1;
      publishStop();
      if (stable_cycles_ >= close_after_stable_cycles_) {
        if (close_gripper_on_arrival_ && !close_sent_) {
          sendGripperGraspForObject();
          close_sent_ = true;
          publishCargoEvent("picked", true);
        }
        done_ = true;
        active_ = false;
        publishStatus("grasp target reached; width-aware gripper command sent", true);
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

  bool useColorTriangulationAfterMinDepth()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return use_color_triangulation_after_min_depth_ && min_depth_reached_;
  }

  std::optional<geometry_msgs::msg::PointStamped> latestDepthObjectInTarget()
  {
    const auto stamp = now();
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!latest_depth_object_in_target_ ||
        latest_depth_object_stamp_.nanoseconds() == 0 ||
        (stamp - latest_depth_object_stamp_).seconds() > max_target_age_s_) {
      return std::nullopt;
    }
    return latest_depth_object_in_target_;
  }

  std::optional<geometry_msgs::msg::PointStamped> estimateObjectPointFromFrontBboxSize()
  {
    Bbox bbox;
    CameraInfo info;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!latest_bbox_ || !latest_camera_info_) {
        return std::nullopt;
      }
      bbox = *latest_bbox_;
      info = *latest_camera_info_;
    }

    if ((now() - bbox.stamp).seconds() > max_target_age_s_ ||
        !shouldHandoffByBboxSize(bbox, info)) {
      return std::nullopt;
    }

    const auto range_m = estimateRangeFromBboxSize(bbox, info);
    if (!range_m) {
      return std::nullopt;
    }

    const double u = bbox.x + 0.5 * bbox.width;
    const double v = bbox.y + 0.5 * bbox.height;
    geometry_msgs::msg::PointStamped object_camera;
    object_camera.header.stamp = builtin_interfaces::msg::Time();
    object_camera.header.frame_id = camera_frame_override_.empty() ? info.frame_id : camera_frame_override_;
    object_camera.point.z = *range_m;
    object_camera.point.x = (u - info.cx) * (*range_m) / info.fx;
    object_camera.point.y = (v - info.cy) * (*range_m) / info.fy;

    try {
      auto object_in_target = tf_buffer_.transform(object_camera, target_frame_);
      rememberObjectDepth(*range_m);
      rememberDepthObjectPoint(object_in_target);
      return object_in_target;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "front bbox size object TF transform failed: %s", ex.what());
      return std::nullopt;
    }
  }

  bool prepareEefColorTriangulation(
    const geometry_msgs::msg::PointStamped & object_in_target,
    std::string * block_reason)
  {
    CameraInfo info;
    std::optional<Bbox> eef_bbox;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (latest_eef_camera_info_) {
        info = *latest_eef_camera_info_;
      } else {
        auto fallback_info = fallbackEefCameraInfo();
        if (!fallback_info) {
          setBlockReason(block_reason, "end-effector camera info");
          return false;
        }
        info = *fallback_info;
      }
      eef_bbox = latest_eef_bbox_;
    }

    publishEefAutoInitEnable(true);

    const auto stamp = now();
    if (eef_bbox && (stamp - eef_bbox->stamp).seconds() <= max_target_age_s_) {
      return true;
    }

    if (auto_init_eef_tracker_from_object_) {
      if (last_eef_init_bbox_stamp_.nanoseconds() == 0 ||
          (stamp - last_eef_init_bbox_stamp_).seconds() >= eef_init_bbox_republish_period_s_) {
        if (publishProjectedEefInitBbox(object_in_target, info)) {
          last_eef_init_bbox_stamp_ = stamp;
        }
      }
    }

    setBlockReason(block_reason, "fresh end-effector bbox for color triangulation");
    return false;
  }

  bool shouldHoldForEefRefinement()
  {
    const auto stamp = now();
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (eef_refinement_requested_) {
      return true;
    }
    return latest_eef_bbox_ &&
      (stamp - latest_eef_bbox_->stamp).seconds() <= max_target_age_s_;
  }

  bool shouldStartEefRefinementByDepth()
  {
    const auto stamp = now();
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (use_color_triangulation_after_min_depth_ && min_depth_reached_) {
      return false;
    }
    return latest_object_depth_m_ &&
      latest_object_depth_stamp_.nanoseconds() != 0 &&
      (stamp - latest_object_depth_stamp_).seconds() <= max_target_age_s_ &&
      *latest_object_depth_m_ <= eef_refinement_start_depth_m_;
  }

  bool prepareEefRefinement(const geometry_msgs::msg::PointStamped & object_in_target)
  {
    CameraInfo info;
    std::optional<Bbox> eef_bbox;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      eef_refinement_requested_ = true;
      if (latest_eef_camera_info_) {
        info = *latest_eef_camera_info_;
      } else {
        auto fallback_info = fallbackEefCameraInfo();
        if (!fallback_info) {
          publishStop();
          publishStatus("waiting for end-effector camera info before near-field refinement");
          return false;
        }
        info = *fallback_info;
      }
      eef_bbox = latest_eef_bbox_;
    }
    publishEefAutoInitEnable(true);

    if (eef_bbox && (now() - eef_bbox->stamp).seconds() <= max_target_age_s_) {
      return true;
    }

    if (!auto_init_eef_tracker_from_object_) {
      publishStop();
      publishStatus("waiting for end-effector visual feature bbox near object");
      return false;
    }

    const auto stamp = now();
    if (last_eef_init_bbox_stamp_.nanoseconds() == 0 ||
        (stamp - last_eef_init_bbox_stamp_).seconds() >= eef_init_bbox_republish_period_s_) {
      if (publishProjectedEefInitBbox(object_in_target, info)) {
        last_eef_init_bbox_stamp_ = stamp;
      }
    }

    publishStop();
    publishStatus("waiting for end-effector visual feature bbox near object");
    return false;
  }

  bool publishProjectedEefInitBbox(
    const geometry_msgs::msg::PointStamped & object_in_target,
    const CameraInfo & info)
  {
    const std::string eef_camera_frame =
      eef_camera_frame_override_.empty() ? info.frame_id : eef_camera_frame_override_;

    geometry_msgs::msg::PointStamped object_latest = object_in_target;
    object_latest.header.stamp = builtin_interfaces::msg::Time();

    geometry_msgs::msg::PointStamped object_in_eef_camera;
    try {
      object_in_eef_camera = tf_buffer_.transform(object_latest, eef_camera_frame);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "end-effector init bbox TF unavailable: %s", ex.what());
      return false;
    }

    const double z_m = object_in_eef_camera.point.z;
    if (!std::isfinite(z_m) || z_m <= 0.01) {
      return false;
    }

    const double u = info.fx * object_in_eef_camera.point.x / z_m + info.cx;
    const double v = info.fy * object_in_eef_camera.point.y / z_m + info.cy;
    if (!std::isfinite(u) || !std::isfinite(v)) {
      return false;
    }

    const double image_width = std::max(1.0, static_cast<double>(info.width));
    const double image_height = std::max(1.0, static_cast<double>(info.height));
    if (u < 0.0 || u >= image_width || v < 0.0 || v >= image_height) {
      return false;
    }

    std::optional<double> measured_width;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      measured_width = latest_object_width_m_;
    }
    const double width_m = measured_width.value_or(gripper_fallback_object_width_m_);
    const double bbox_w = clampValue(
      eef_init_bbox_padding_scale_ * width_m * info.fx / z_m,
      eef_init_bbox_min_size_px_, eef_init_bbox_max_size_px_);
    const double bbox_h = clampValue(
      eef_init_bbox_padding_scale_ * object_height_m_ * info.fy / z_m,
      eef_init_bbox_min_size_px_, eef_init_bbox_max_size_px_);

    const double x = clampValue(u - 0.5 * bbox_w, 0.0, image_width - 1.0);
    const double y = clampValue(v - 0.5 * bbox_h, 0.0, image_height - 1.0);
    const double w = std::min(bbox_w, image_width - x);
    const double h = std::min(bbox_h, image_height - y);
    if (w < 2.0 || h < 2.0) {
      return false;
    }

    std_msgs::msg::Float32MultiArray msg;
    msg.data = {
      static_cast<float>(x),
      static_cast<float>(y),
      static_cast<float>(w),
      static_cast<float>(h)};
    eef_init_bbox_pub_->publish(msg);
    return true;
  }

  void updateEefRefinement(const geometry_msgs::msg::PointStamped & object_in_target)
  {
    if (!prepareEefRefinement(object_in_target)) {
      return;
    }

    Bbox bbox;
    CameraInfo info;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!latest_eef_bbox_) {
        publishStop();
        publishStatus("waiting for end-effector visual feature bbox");
        return;
      }
      bbox = *latest_eef_bbox_;
      if (latest_eef_camera_info_) {
        info = *latest_eef_camera_info_;
      } else {
        auto fallback_info = fallbackEefCameraInfo();
        if (!fallback_info) {
          publishStop();
          publishStatus("waiting for end-effector camera info");
          return;
        }
        info = *fallback_info;
      }
    }

    if ((now() - bbox.stamp).seconds() > max_target_age_s_) {
      publishStop();
      publishStatus("waiting for fresh end-effector visual feature bbox");
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

    const double bbox_u = bbox.x + 0.5 * bbox.width;
    const double bbox_v = bbox.y + 0.5 * bbox.height;
    const bool projected_object_range_valid =
      std::isfinite(object_in_eef_camera.point.z) &&
      object_in_eef_camera.point.z > 0.0;
    const double projected_u = projected_object_range_valid ?
      info.fx * object_in_eef_camera.point.x / object_in_eef_camera.point.z + info.cx :
      std::numeric_limits<double>::quiet_NaN();
    const double projected_v = projected_object_range_valid ?
      info.fy * object_in_eef_camera.point.y / object_in_eef_camera.point.z + info.cy :
      std::numeric_limits<double>::quiet_NaN();
    const bool projected_center_valid =
      std::isfinite(projected_u) && std::isfinite(projected_v) &&
      projected_u >= 0.0 && projected_u < static_cast<double>(info.width) &&
      projected_v >= 0.0 && projected_v < static_cast<double>(info.height);
    const double u = projected_center_valid ? projected_u : bbox_u;
    const double v = projected_center_valid ? projected_v : bbox_v;
    const double err_u_px = u - info.cx;
    const double err_v_px = v - info.cy;
    const double z_m = projected_center_valid ?
      clampValue(
        object_in_eef_camera.point.z,
        eef_final_depth_m_,
        eef_refinement_start_depth_m_) :
      eef_refinement_switch_distance_m_;
    const double lateral_m = err_u_px * z_m / info.fx;
    const double vertical_m = err_v_px * z_m / info.fy;

    const double center_tolerance_px = scaledEefTolerancePx(eef_center_tolerance_px_, info);
    const double close_tolerance_px = scaledEefTolerancePx(eef_close_tolerance_px_, info);
    const bool centered =
      std::abs(err_u_px) <= center_tolerance_px &&
      std::abs(err_v_px) <= center_tolerance_px;
    const bool close_ready =
      std::abs(err_u_px) <= close_tolerance_px &&
      std::abs(err_v_px) <= close_tolerance_px;

    if (close_ready) {
      stable_cycles_ += 1;
      publishStop();
      if (stable_cycles_ >= close_after_stable_cycles_) {
        if (close_gripper_on_arrival_ && !close_sent_) {
          sendGripperGraspForObject();
          close_sent_ = true;
          publishCargoEvent("picked", true);
        }
        done_ = true;
        active_ = false;
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          eef_refinement_requested_ = false;
        }
        publishStatus("eef camera refined grasp reached; width-aware gripper command sent", true);
      } else {
        std::ostringstream hold_status;
        hold_status << "eef camera close-ready; holding before closing"
                    << " pixel_err=(" << err_u_px << ", " << err_v_px << ")"
                    << " strict_centered=" << (centered ? "true" : "false");
        publishStatus(hold_status.str());
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
    cmd.twist.linear.z = 0.0;
    twist_pub_->publish(cmd);

    std::ostringstream status;
    status << "eef refine pixel_err=(" << err_u_px << ", " << err_v_px << ")"
           << " feature_err=(" << (bbox_u - u) << ", " << (bbox_v - v) << ")"
           << " feature_source=" << (projected_center_valid ? "front_projection" : "eef_bbox")
           << " range_cmd=disabled"
           << " close_tol_px=" << close_tolerance_px
           << " eef_resolution=" << info.width << "x" << info.height
           << " cmd_frame=" << eef_camera_frame;
    publishStatus(status.str());
  }

  bool isDepthUnavailableReason(const std::string & reason) const
  {
    return reason.rfind("valid depth inside bbox", 0) == 0 ||
      reason.rfind("depth image on ", 0) == 0;
  }

  std::optional<geometry_msgs::msg::PointStamped> estimateObjectPointByTriangulation(
    const geometry_msgs::msg::TransformStamped & eef_tf,
    std::string * block_reason,
    bool * command_published)
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      eef_refinement_requested_ = true;
    }
    publishEefAutoInitEnable(true);

    if (stage_ != GraspStage::EEF_REFINE &&
        !moveArmToTriangulationPose(eef_tf, command_published)) {
      setBlockReason(block_reason, "arm extension for stereo triangulation");
      return std::nullopt;
    }

    return triangulateObjectPoint(block_reason);
  }

  bool moveArmToObjectPregraspPose(
    const geometry_msgs::msg::TransformStamped & eef_tf,
    const geometry_msgs::msg::PointStamped & object_in_target,
    bool * command_published)
  {
    const double target_x = object_in_target.point.x - object_pregrasp_standoff_m_ + grasp_offset_x_;
    const double target_y = object_in_target.point.y + grasp_offset_y_;

    const double eef_x = eef_tf.transform.translation.x;
    const double eef_y = eef_tf.transform.translation.y;
    const double eef_z = eef_tf.transform.translation.z;
    const double horizontal_err = vectorNorm(target_x - eef_x, target_y - eef_y, 0.0);
    const double safe_target_z = std::max(eef_z, object_pregrasp_min_z_m_);
    if (!object_pregrasp_horizontal_done_ &&
        horizontal_err <= triangulation_extend_tolerance_m_ &&
        std::abs(safe_target_z - eef_z) <= triangulation_extend_tolerance_m_) {
      object_pregrasp_horizontal_done_ = true;
    }
    if (object_pregrasp_horizontal_done_ && !object_pregrasp_enable_lowering_) {
      return true;
    }

    const double target_z = object_pregrasp_horizontal_done_ && object_pregrasp_enable_lowering_ ?
      std::max(
        object_in_target.point.z + grasp_offset_z_ + object_pregrasp_lower_standoff_m_,
        object_pregrasp_min_lower_z_m_) :
      safe_target_z;
    const double err_x = target_x - eef_x;
    const double err_y = target_y - eef_y;
    const double err_z = target_z - eef_z;
    const double err_norm = vectorNorm(err_x, err_y, err_z);

    if (err_norm <= triangulation_extend_tolerance_m_) {
      return true;
    }

    stage_ = GraspStage::TRIANGULATION_EXTEND;
    stable_cycles_ = 0;

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = target_frame_;
    cmd.twist.linear.x = clampValue(
      triangulation_extend_gain_ * err_x,
      -triangulation_extend_max_speed_, triangulation_extend_max_speed_);
    cmd.twist.linear.y = clampValue(
      triangulation_extend_gain_ * err_y,
      -triangulation_extend_max_speed_, triangulation_extend_max_speed_);
    cmd.twist.linear.z = clampValue(
      triangulation_extend_gain_ * err_z,
      -triangulation_extend_max_speed_, triangulation_extend_max_speed_);
    twist_pub_->publish(cmd);
    if (command_published) {
      *command_published = true;
    }

    std::ostringstream status;
    status << (object_pregrasp_horizontal_done_ ?
      "lowering arm toward depth object: target=(" :
      "extending arm horizontally toward depth object: target=(")
           << target_x << ", " << target_y << ", " << target_z
           << ") err=(" << err_x << ", " << err_y << ", " << err_z
           << ") norm=" << err_norm;
    publishStatus(status.str());
    return false;
  }

  bool moveArmToTriangulationPose(
    const geometry_msgs::msg::TransformStamped & eef_tf,
    bool * command_published)
  {
    const double eef_x = eef_tf.transform.translation.x;
    const double eef_y = eef_tf.transform.translation.y;
    const double eef_z = eef_tf.transform.translation.z;
    const double err_x = triangulation_extend_x_m_ - eef_x;
    const double err_y = triangulation_extend_y_m_ - eef_y;
    const double err_z = triangulation_extend_z_m_ - eef_z;
    const double err_norm = vectorNorm(err_x, err_y, err_z);

    if (err_norm <= triangulation_extend_tolerance_m_) {
      if (stage_ == GraspStage::TRIANGULATION_EXTEND) {
        stage_ = GraspStage::DEPTH_APPROACH;
      }
      return true;
    }

    stage_ = GraspStage::TRIANGULATION_EXTEND;
    stable_cycles_ = 0;

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = target_frame_;
    cmd.twist.linear.x = clampValue(
      triangulation_extend_gain_ * err_x,
      -triangulation_extend_max_speed_, triangulation_extend_max_speed_);
    cmd.twist.linear.y = clampValue(
      triangulation_extend_gain_ * err_y,
      -triangulation_extend_max_speed_, triangulation_extend_max_speed_);
    cmd.twist.linear.z = clampValue(
      triangulation_extend_gain_ * err_z,
      -triangulation_extend_max_speed_, triangulation_extend_max_speed_);
    twist_pub_->publish(cmd);
    if (command_published) {
      *command_published = true;
    }

    std::ostringstream status;
    status << "extending arm for stereo triangulation: err=("
           << err_x << ", " << err_y << ", " << err_z
           << ") norm=" << err_norm;
    publishStatus(status.str());
    return false;
  }

  std::optional<geometry_msgs::msg::PointStamped> triangulateObjectPoint(
    std::string * block_reason,
    bool publish_triangulation_status = true)
  {
    Bbox front_bbox;
    Bbox eef_bbox;
    CameraInfo front_info;
    CameraInfo eef_info;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!latest_bbox_) {
        setBlockReason(block_reason, bboxInputDescription());
        return std::nullopt;
      }
      if (!latest_camera_info_) {
        setBlockReason(block_reason, "front camera info on " + camera_info_topic_);
        return std::nullopt;
      }
      if (!latest_eef_bbox_) {
        setBlockReason(block_reason, "end-effector bbox for stereo triangulation");
        return std::nullopt;
      }
      front_bbox = *latest_bbox_;
      eef_bbox = *latest_eef_bbox_;
      front_info = *latest_camera_info_;
      if (latest_eef_camera_info_) {
        eef_info = *latest_eef_camera_info_;
      } else {
        auto fallback_info = fallbackEefCameraInfo();
        if (!fallback_info) {
          setBlockReason(block_reason, "end-effector camera info");
          return std::nullopt;
        }
        eef_info = *fallback_info;
      }
    }

    const auto stamp = now();
    if ((stamp - front_bbox.stamp).seconds() > max_target_age_s_) {
      setBlockReason(block_reason, "fresh " + bboxInputDescription());
      return std::nullopt;
    }
    if ((stamp - eef_bbox.stamp).seconds() > max_target_age_s_) {
      setBlockReason(block_reason, "fresh end-effector bbox for stereo triangulation");
      return std::nullopt;
    }

    const std::string front_frame =
      camera_frame_override_.empty() ? front_info.frame_id : camera_frame_override_;
    const std::string eef_frame =
      eef_camera_frame_override_.empty() ? eef_info.frame_id : eef_camera_frame_override_;
    const double front_u = front_bbox.x + 0.5 * front_bbox.width;
    const double front_v = front_bbox.y + 0.5 * front_bbox.height;
    const double eef_u = eef_bbox.x + 0.5 * eef_bbox.width;
    const double eef_v = eef_bbox.y + 0.5 * eef_bbox.height;

    auto front_ray = cameraPixelRayInTarget(front_frame, front_info, front_u, front_v, block_reason);
    if (!front_ray) {
      return std::nullopt;
    }
    auto eef_ray = cameraPixelRayInTarget(eef_frame, eef_info, eef_u, eef_v, block_reason);
    if (!eef_ray) {
      return std::nullopt;
    }

    const tf2::Vector3 w0 = front_ray->origin - eef_ray->origin;
    const double a = front_ray->direction.dot(front_ray->direction);
    const double b = front_ray->direction.dot(eef_ray->direction);
    const double c = eef_ray->direction.dot(eef_ray->direction);
    const double d = front_ray->direction.dot(w0);
    const double e = eef_ray->direction.dot(w0);
    const double denom = a * c - b * b;
    if (std::abs(denom) < 1e-6) {
      setBlockReason(block_reason, "non-parallel camera rays for stereo triangulation");
      return std::nullopt;
    }

    const double front_range = (b * e - c * d) / denom;
    const double eef_range = (a * e - b * d) / denom;
    if (front_range < triangulation_min_range_m_ || front_range > triangulation_max_range_m_ ||
        eef_range < triangulation_min_range_m_ || eef_range > triangulation_max_range_m_) {
      std::ostringstream reason;
      reason << "triangulation range front=" << front_range << " eef=" << eef_range;
      setBlockReason(block_reason, reason.str());
      return std::nullopt;
    }

    const tf2::Vector3 front_point = front_ray->origin + front_ray->direction * front_range;
    const tf2::Vector3 eef_point = eef_ray->origin + eef_ray->direction * eef_range;
    const double ray_gap = (front_point - eef_point).length();
    if (ray_gap > triangulation_max_ray_gap_m_) {
      std::ostringstream reason;
      reason << "triangulation ray gap " << ray_gap;
      setBlockReason(block_reason, reason.str());
      return std::nullopt;
    }

    const tf2::Vector3 object = (front_point + eef_point) * 0.5;
    if (!std::isfinite(object.x()) || !std::isfinite(object.y()) || !std::isfinite(object.z())) {
      setBlockReason(block_reason, "finite stereo triangulation result");
      return std::nullopt;
    }

    rememberMeasuredObjectWidth(eef_bbox.width * eef_range / eef_info.fx);

    geometry_msgs::msg::PointStamped point;
    point.header.stamp = stamp;
    point.header.frame_id = target_frame_;
    point.point.x = object.x();
    point.point.y = object.y();
    point.point.z = object.z();

    if (publish_triangulation_status) {
      std::ostringstream status;
      status << "stereo triangulation object=(" << point.point.x << ", "
             << point.point.y << ", " << point.point.z
             << ") ray_gap=" << ray_gap;
      publishStatus(status.str());
    }
    return point;
  }

  std::optional<Ray> cameraPixelRayInTarget(
    const std::string & camera_frame,
    const CameraInfo & info,
    double u,
    double v,
    std::string * block_reason)
  {
    if (camera_frame.empty()) {
      setBlockReason(block_reason, "camera frame for stereo triangulation");
      return std::nullopt;
    }

    geometry_msgs::msg::TransformStamped transform_msg;
    try {
      transform_msg = tf_buffer_.lookupTransform(target_frame_, camera_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      setBlockReason(block_reason, "TF from " + camera_frame + " to " + target_frame_);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "stereo triangulation TF unavailable: %s", ex.what());
      return std::nullopt;
    }

    tf2::Transform transform;
    tf2::fromMsg(transform_msg.transform, transform);

    tf2::Vector3 ray_camera((u - info.cx) / info.fx, (v - info.cy) / info.fy, 1.0);
    if (ray_camera.length2() < 1e-9) {
      setBlockReason(block_reason, "valid pixel ray for stereo triangulation");
      return std::nullopt;
    }
    ray_camera.normalize();

    Ray ray;
    ray.origin = transform.getOrigin();
    ray.direction = transform.getBasis() * ray_camera;
    if (ray.direction.length2() < 1e-9) {
      setBlockReason(block_reason, "valid target-frame ray for stereo triangulation");
      return std::nullopt;
    }
    ray.direction.normalize();
    return ray;
  }

  std::optional<geometry_msgs::msg::PointStamped> estimateObjectPoint(
    std::string * block_reason = nullptr)
  {
    Bbox bbox;
    CameraInfo info;
    sensor_msgs::msg::Image::ConstSharedPtr depth;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!latest_bbox_) {
        setBlockReason(block_reason, bboxInputDescription());
        return std::nullopt;
      }
      if (!latest_camera_info_) {
        setBlockReason(block_reason, "camera info on " + camera_info_topic_);
        return std::nullopt;
      }
      if (!latest_depth_) {
        setBlockReason(block_reason, "depth image on " + depth_topic_);
        return std::nullopt;
      }
      if (use_color_triangulation_after_min_depth_ && min_depth_reached_) {
        setBlockReason(block_reason, "color triangulation after minimum depth");
        return std::nullopt;
      }
      bbox = *latest_bbox_;
      info = *latest_camera_info_;
      depth = latest_depth_;
    }

    if ((now() - bbox.stamp).seconds() > max_target_age_s_) {
      setBlockReason(block_reason, "fresh " + bboxInputDescription());
      return std::nullopt;
    }

    const double u = bbox.x + 0.5 * bbox.width;
    const double v = bbox.y + 0.5 * bbox.height;
    if (use_color_triangulation_after_min_depth_ && shouldHandoffByBboxSize(bbox, info)) {
      const double handoff_depth =
        estimateRangeFromBboxSize(bbox, info).value_or(eef_refinement_start_depth_m_);
      rememberObjectDepth(handoff_depth);
      maybePreEnableEefYolo(handoff_depth);

      geometry_msgs::msg::PointStamped object_camera;
      object_camera.header.stamp = depth->header.stamp;
      object_camera.header.frame_id =
        camera_frame_override_.empty() ? info.frame_id : camera_frame_override_;
      object_camera.point.z = handoff_depth;
      object_camera.point.x = (u - info.cx) * handoff_depth / info.fx;
      object_camera.point.y = (v - info.cy) * handoff_depth / info.fy;

      try {
        rememberDepthObjectPoint(tf_buffer_.transform(object_camera, target_frame_));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "bbox-size handoff TF transform failed: %s", ex.what());
      }

      std::ostringstream reason;
      reason << "minimum depth reached by bbox size"
             << " area_ratio=" << bboxAreaRatio(bbox, info)
             << " height_ratio=" << bboxHeightRatio(bbox, info)
             << " estimated_range=" << handoff_depth
             << "; switching to color triangulation";
      setBlockReason(block_reason, reason.str());
      return std::nullopt;
    }

    auto depth_m = robustDepthInBbox(*depth, info, bbox);
    if (!depth_m) {
      depth_m = medianDepthAt(*depth, info, u, v);
    }
    if (!depth_m) {
      if (use_color_triangulation_after_min_depth_) {
        const auto near_limit_depth = nearLimitDepthInBbox(*depth, info, bbox);
        if (near_limit_depth) {
          rememberObjectDepth(*near_limit_depth);
          maybePreEnableEefYolo(*near_limit_depth);

          geometry_msgs::msg::PointStamped object_camera;
          object_camera.header.stamp = depth->header.stamp;
          object_camera.header.frame_id =
            camera_frame_override_.empty() ? info.frame_id : camera_frame_override_;
          object_camera.point.z = *near_limit_depth;
          object_camera.point.x = (u - info.cx) * (*near_limit_depth) / info.fx;
          object_camera.point.y = (v - info.cy) * (*near_limit_depth) / info.fy;

          try {
            rememberDepthObjectPoint(tf_buffer_.transform(object_camera, target_frame_));
          } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "near-limit object TF transform failed: %s", ex.what());
          }

          std::ostringstream reason;
          reason << "minimum depth reached by near-limit sample "
                 << *near_limit_depth << "m; switching to color triangulation";
          setBlockReason(block_reason, reason.str());
          return std::nullopt;
        }
      }

      std::ostringstream reason;
      reason << "valid depth inside bbox in ["
             << min_valid_depth_m_ << ", " << max_valid_depth_m_ << "] m";
      setBlockReason(block_reason, reason.str());
      return std::nullopt;
    }
    rememberObjectDepth(*depth_m);
    maybePreEnableEefYolo(*depth_m);
    rememberMeasuredObjectWidth(bbox.width * (*depth_m) / info.fx);

    geometry_msgs::msg::PointStamped object_camera;
    object_camera.header.stamp = depth->header.stamp;
    object_camera.header.frame_id = camera_frame_override_.empty() ? info.frame_id : camera_frame_override_;
    object_camera.point.z = *depth_m;
    object_camera.point.x = (u - info.cx) * (*depth_m) / info.fx;
    object_camera.point.y = (v - info.cy) * (*depth_m) / info.fy;

    try {
      auto object_in_target = tf_buffer_.transform(object_camera, target_frame_);
      rememberDepthObjectPoint(object_in_target);
      return object_in_target;
    } catch (const tf2::TransformException & ex) {
      setBlockReason(block_reason, "TF from " + object_camera.header.frame_id + " to " + target_frame_);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "object TF transform failed: %s", ex.what());
      return std::nullopt;
    }
  }

  void setBlockReason(std::string * block_reason, const std::string & reason) const
  {
    if (block_reason) {
      *block_reason = reason;
    }
  }

  std::string bboxInputDescription() const
  {
    std::string description = "front bbox on " + bbox_topic_;
    if (use_fallback_bbox_for_control_ && !fallback_bbox_topic_.empty()) {
      description += " or fallback bbox on " + fallback_bbox_topic_;
    }
    return description;
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

  std::optional<double> robustDepthInBbox(
    const sensor_msgs::msg::Image & depth,
    const CameraInfo & info,
    const Bbox & bbox) const
  {
    const double image_width =
      info.width > 0 ? static_cast<double>(info.width) : static_cast<double>(depth.width);
    const double image_height =
      info.height > 0 ? static_cast<double>(info.height) : static_cast<double>(depth.height);

    const int col_begin = clampValue(
      static_cast<int>(std::floor(bbox.x * static_cast<double>(depth.width) /
      std::max(1.0, image_width))),
      0,
      static_cast<int>(depth.width) - 1);
    const int col_end = clampValue(
      static_cast<int>(std::ceil((bbox.x + bbox.width) * static_cast<double>(depth.width) /
      std::max(1.0, image_width))),
      0,
      static_cast<int>(depth.width) - 1);
    const int row_begin = clampValue(
      static_cast<int>(std::floor(bbox.y * static_cast<double>(depth.height) /
      std::max(1.0, image_height))),
      0,
      static_cast<int>(depth.height) - 1);
    const int row_end = clampValue(
      static_cast<int>(std::ceil((bbox.y + bbox.height) * static_cast<double>(depth.height) /
      std::max(1.0, image_height))),
      0,
      static_cast<int>(depth.height) - 1);

    if (row_end < row_begin || col_end < col_begin) {
      return std::nullopt;
    }

    const int roi_width = std::max(1, col_end - col_begin + 1);
    const int roi_height = std::max(1, row_end - row_begin + 1);
    const int step = std::max(1, std::min(roi_width, roi_height) / 24);

    std::vector<double> samples;
    samples.reserve(static_cast<size_t>((roi_width / step + 1) * (roi_height / step + 1)));
    for (int row = row_begin; row <= row_end; row += step) {
      for (int col = col_begin; col <= col_end; col += step) {
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
    const size_t index = std::min(samples.size() - 1, samples.size() / 4);
    return samples[index];
  }

  std::optional<double> nearLimitDepthInBbox(
    const sensor_msgs::msg::Image & depth,
    const CameraInfo & info,
    const Bbox & bbox) const
  {
    const double image_width =
      info.width > 0 ? static_cast<double>(info.width) : static_cast<double>(depth.width);
    const double image_height =
      info.height > 0 ? static_cast<double>(info.height) : static_cast<double>(depth.height);

    const int col_begin = clampValue(
      static_cast<int>(std::floor(bbox.x * static_cast<double>(depth.width) /
      std::max(1.0, image_width))),
      0,
      static_cast<int>(depth.width) - 1);
    const int col_end = clampValue(
      static_cast<int>(std::ceil((bbox.x + bbox.width) * static_cast<double>(depth.width) /
      std::max(1.0, image_width))),
      0,
      static_cast<int>(depth.width) - 1);
    const int row_begin = clampValue(
      static_cast<int>(std::floor(bbox.y * static_cast<double>(depth.height) /
      std::max(1.0, image_height))),
      0,
      static_cast<int>(depth.height) - 1);
    const int row_end = clampValue(
      static_cast<int>(std::ceil((bbox.y + bbox.height) * static_cast<double>(depth.height) /
      std::max(1.0, image_height))),
      0,
      static_cast<int>(depth.height) - 1);

    if (row_end < row_begin || col_end < col_begin) {
      return std::nullopt;
    }

    const int roi_width = std::max(1, col_end - col_begin + 1);
    const int roi_height = std::max(1, row_end - row_begin + 1);
    const int step = std::max(1, std::min(roi_width, roi_height) / 24);
    const double handoff_depth = eef_refinement_start_depth_m_ + min_depth_handoff_margin_m_;

    std::vector<double> samples;
    samples.reserve(static_cast<size_t>((roi_width / step + 1) * (roi_height / step + 1)));
    for (int row = row_begin; row <= row_end; row += step) {
      for (int col = col_begin; col <= col_end; col += step) {
        const auto meters = depthPixelMeters(depth, row, col);
        if (meters && *meters > 0.02 && *meters <= handoff_depth) {
          samples.push_back(*meters);
        }
      }
    }

    if (samples.size() < 5U) {
      return std::nullopt;
    }

    std::sort(samples.begin(), samples.end());
    return samples[samples.size() / 2];
  }

  std::optional<double> estimateRangeFromBboxSize(
    const Bbox & bbox,
    const CameraInfo & info) const
  {
    if (bbox.height <= 2.0 || info.fy <= 0.0 || object_height_m_ <= 0.0) {
      return std::nullopt;
    }

    const double range_m = object_height_m_ * info.fy / bbox.height;
    if (!std::isfinite(range_m) || range_m <= 0.0) {
      return std::nullopt;
    }

    return clampValue(range_m, color_triangulation_min_object_x_m_, eef_refinement_start_depth_m_);
  }

  double bboxAreaRatio(const Bbox & bbox, const CameraInfo & info) const
  {
    const double image_width = std::max(1.0, static_cast<double>(info.width));
    const double image_height = std::max(1.0, static_cast<double>(info.height));
    return std::max(0.0, bbox.width) * std::max(0.0, bbox.height) /
      (image_width * image_height);
  }

  double bboxHeightRatio(const Bbox & bbox, const CameraInfo & info) const
  {
    const double image_height = std::max(1.0, static_cast<double>(info.height));
    return std::max(0.0, bbox.height) / image_height;
  }

  bool shouldHandoffByBboxSize(const Bbox & bbox, const CameraInfo & info) const
  {
    if (min_depth_handoff_bbox_area_ratio_ <= 0.0 &&
        min_depth_handoff_bbox_height_ratio_ <= 0.0) {
      return false;
    }

    const double area_ratio = bboxAreaRatio(bbox, info);
    const double height_ratio = bboxHeightRatio(bbox, info);
    return
      (min_depth_handoff_bbox_area_ratio_ > 0.0 &&
      area_ratio >= min_depth_handoff_bbox_area_ratio_) ||
      (min_depth_handoff_bbox_height_ratio_ > 0.0 &&
      height_ratio >= min_depth_handoff_bbox_height_ratio_);
  }

  double scaledEefTolerancePx(double tolerance_px, const CameraInfo & info) const
  {
    const double baseline_width = static_cast<double>(std::max(1, eef_camera_fallback_width_px_));
    const double baseline_height = static_cast<double>(std::max(1, eef_camera_fallback_height_px_));
    const double width_scale = static_cast<double>(std::max<std::uint32_t>(1U, info.width)) / baseline_width;
    const double height_scale = static_cast<double>(std::max<std::uint32_t>(1U, info.height)) / baseline_height;
    return std::max(1.0, tolerance_px * std::min(width_scale, height_scale));
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

  void rememberMeasuredObjectWidth(double width_m)
  {
    if (!std::isfinite(width_m) ||
        width_m < gripper_min_measured_object_width_m_ ||
        width_m > gripper_max_measured_object_width_m_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ignoring object width estimate %.3f m outside [%.3f, %.3f] m",
        width_m, gripper_min_measured_object_width_m_, gripper_max_measured_object_width_m_);
      return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_object_width_m_ = width_m;
  }

  void rememberObjectDepth(double depth_m)
  {
    if (!std::isfinite(depth_m)) {
      return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_object_depth_m_ = depth_m;
    latest_object_depth_stamp_ = now();
    if (use_color_triangulation_after_min_depth_ &&
        depth_m <= eef_refinement_start_depth_m_ + min_depth_handoff_margin_m_) {
      min_depth_reached_ = true;
    }
  }

  void maybePreEnableEefYolo(double depth_m)
  {
    if (!use_eef_refinement_ ||
        !use_color_triangulation_after_min_depth_ ||
        !std::isfinite(depth_m) ||
        depth_m > eef_yolo_pre_enable_depth_m_) {
      return;
    }

    publishEefAutoInitEnable(true);
  }

  void rememberDepthObjectPoint(const geometry_msgs::msg::PointStamped & object_in_target)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_depth_object_in_target_ = object_in_target;
    latest_depth_object_stamp_ = now();
  }

  double gripperPositionForGap(double gap_m) const
  {
    const double joint_position =
      0.5 * std::max(0.0, gap_m) - gripper_finger_home_half_gap_m_;
    return clampValue(joint_position, gripper_min_position_, gripper_max_position_);
  }

  GripperTarget makeGripperTarget(bool open)
  {
    if (!gripper_width_control_enabled_) {
      return GripperTarget{
        open ? gripper_open_position_ : gripper_close_position_,
        0.0,
        false};
    }

    std::optional<double> measured_width;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      measured_width = latest_object_width_m_;
    }

    const double object_width_m =
      measured_width.value_or(gripper_fallback_object_width_m_);
    const double target_gap_m = open ?
      object_width_m + gripper_pre_grasp_clearance_m_ :
      std::max(0.0, object_width_m - gripper_grasp_compression_m_);

    return GripperTarget{
      gripperPositionForGap(target_gap_m),
      object_width_m,
      measured_width.has_value()};
  }

  void sendGripperOpenForObject()
  {
    const auto target = makeGripperTarget(true);
    if (gripper_width_control_enabled_) {
      RCLCPP_INFO(
        get_logger(),
        "gripper open target: object_width=%.3f m (%s), position=%.4f m",
        target.object_width_m,
        target.measured ? "measured" : "fallback",
        target.position);
    }
    sendGripper(target.position);
  }

  void sendGripperGraspForObject()
  {
    const auto target = makeGripperTarget(false);
    if (gripper_width_control_enabled_) {
      RCLCPP_INFO(
        get_logger(),
        "gripper grasp target: object_width=%.3f m (%s), position=%.4f m",
        target.object_width_m,
        target.measured ? "measured" : "fallback",
        target.position);
    }
    sendGripper(target.position);
  }

  void sendGripper(double position)
  {
    position = clampValue(position, gripper_min_position_, gripper_max_position_);
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
    if (!servo_start_client_->wait_for_service(std::chrono::milliseconds(500))) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "MoveIt Servo start service unavailable: /servo_node/start_servo");
      return;
    }

    servo_start_client_->async_send_request(
      std::make_shared<Trigger::Request>(),
      [this](rclcpp::Client<Trigger>::SharedFuture future) {
        const auto response = future.get();
        if (!response->success) {
          RCLCPP_WARN(
            get_logger(), "MoveIt Servo start request returned false: %s",
            response->message.c_str());
        }
      });
  }

  void publishStop()
  {
    geometry_msgs::msg::TwistStamped stop;
    stop.header.stamp = now();
    stop.header.frame_id = target_frame_;
    twist_pub_->publish(stop);
  }

  void publishEefAutoInitEnable(bool enabled)
  {
    if (!eef_auto_init_enable_pub_) {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    eef_auto_init_enable_pub_->publish(msg);
  }

  void publishBaseHold(bool enabled)
  {
    if (!base_hold_pub_) {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    base_hold_pub_->publish(msg);
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

  std::optional<CameraInfo> fallbackEefCameraInfo() const
  {
    if (!allow_eef_camera_info_fallback_) {
      return std::nullopt;
    }

    CameraInfo info;
    info.fx = eef_camera_fallback_fx_;
    info.fy = eef_camera_fallback_fy_;
    info.width = static_cast<std::uint32_t>(eef_camera_fallback_width_px_);
    info.height = static_cast<std::uint32_t>(eef_camera_fallback_height_px_);
    info.cx = 0.5 * static_cast<double>(info.width);
    info.cy = 0.5 * static_cast<double>(info.height);
    info.frame_id =
      eef_camera_frame_override_.empty() ? "eef_usb_camera_optical_frame" :
      eef_camera_frame_override_;
    return info;
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
  std::string eef_init_bbox_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string eef_camera_info_topic_;
  std::string eef_auto_init_enable_topic_;
  std::string base_hold_topic_;
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
  bool use_fallback_bbox_for_control_{false};
  bool start_servo_on_start_{true};
  bool open_gripper_on_start_{true};
  bool close_gripper_on_arrival_{true};
  bool use_eef_refinement_{true};
  bool wait_for_base_approach_{false};
  bool auto_init_eef_tracker_from_object_{true};
  bool allow_eef_camera_info_fallback_{true};
  bool use_depthless_triangulation_{false};
  bool use_color_triangulation_after_min_depth_{false};
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
  double eef_refinement_start_depth_m_{0.47};
  double eef_yolo_pre_enable_depth_m_{0.47};
  double min_depth_handoff_margin_m_{0.02};
  double min_depth_handoff_bbox_area_ratio_{0.30};
  double min_depth_handoff_bbox_height_ratio_{0.75};
  double eef_refinement_start_object_x_m_{0.50};
  double arm_start_max_error_m_{0.40};
  double arm_start_max_object_x_m_{0.60};
  double object_height_m_{0.10};
  double eef_init_bbox_min_size_px_{32.0};
  double eef_init_bbox_max_size_px_{180.0};
  double eef_init_bbox_padding_scale_{1.6};
  double eef_init_bbox_republish_period_s_{0.5};
  double eef_final_depth_m_{0.08};
  double object_pregrasp_standoff_m_{0.08};
  double object_pregrasp_min_z_m_{0.50};
  double object_pregrasp_lower_standoff_m_{0.02};
  double object_pregrasp_min_lower_z_m_{0.12};
  bool object_pregrasp_enable_lowering_{false};
  double eef_center_tolerance_px_{18.0};
  double eef_close_tolerance_px_{18.0};
  double eef_depth_tolerance_m_{0.018};
  double eef_refine_lateral_gain_{0.8};
  double eef_refine_depth_gain_{0.5};
  double eef_refine_max_linear_speed_{0.012};
  double triangulation_extend_x_m_{0.25};
  double triangulation_extend_y_m_{0.0};
  double triangulation_extend_z_m_{0.12};
  double triangulation_extend_tolerance_m_{0.025};
  double triangulation_extend_gain_{0.7};
  double triangulation_extend_max_speed_{0.015};
  double triangulation_min_range_m_{0.06};
  double triangulation_max_range_m_{1.0};
  double triangulation_max_ray_gap_m_{0.08};
  double color_triangulation_base_stop_object_x_m_{0.30};
  double color_triangulation_min_object_x_m_{0.05};
  int eef_camera_fallback_width_px_{640};
  int eef_camera_fallback_height_px_{480};
  double eef_camera_fallback_fx_{554.0};
  double eef_camera_fallback_fy_{554.0};
  double gripper_open_position_{0.025};
  double gripper_close_position_{-0.015};
  double gripper_max_effort_{-1.0};
  bool gripper_width_control_enabled_{true};
  double gripper_fallback_object_width_m_{0.06};
  double gripper_finger_home_half_gap_m_{0.021};
  double gripper_pre_grasp_clearance_m_{0.012};
  double gripper_grasp_compression_m_{0.002};
  double gripper_min_position_{-0.010};
  double gripper_max_position_{0.019};
  double gripper_min_measured_object_width_m_{0.01};
  double gripper_max_measured_object_width_m_{0.08};
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
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr eef_init_bbox_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr eef_auto_init_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr base_hold_pub_;
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
  std::optional<double> latest_object_width_m_;
  std::optional<double> latest_object_depth_m_;
  rclcpp::Time latest_object_depth_stamp_;
  std::optional<geometry_msgs::msg::PointStamped> latest_depth_object_in_target_;
  rclcpp::Time latest_depth_object_stamp_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;

  bool active_{false};
  bool done_{false};
  bool open_sent_{false};
  bool close_sent_{false};
  bool eef_refinement_requested_{false};
  bool object_pregrasp_horizontal_done_{false};
  bool min_depth_reached_{false};
  GraspStage stage_{GraspStage::DEPTH_APPROACH};
  int stable_cycles_{0};
  rclcpp::Time last_status_stamp_;
  rclcpp::Time last_eef_init_bbox_stamp_;
};

}  // namespace mp_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mp_control::MpControlNode>());
  rclcpp::shutdown();
  return 0;
}
