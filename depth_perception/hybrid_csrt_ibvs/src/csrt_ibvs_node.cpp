#include "hybrid_csrt_ibvs/csrt_ibvs_node.hpp"

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

#ifndef HYBRID_CSRT_IBVS_HAS_OPENCV_TRACKING
#include <opencv2/video/tracking.hpp>
#endif

namespace hybrid_csrt_ibvs
{
namespace
{
constexpr char kBgr8[] = "bgr8";

double rectIou(const cv::Rect & a, const cv::Rect & b)
{
  const cv::Rect intersection = a & b;
  const double intersection_area = static_cast<double>(intersection.area());
  const double union_area = static_cast<double>(a.area() + b.area()) - intersection_area;
  if (union_area <= 0.0) {
    return 0.0;
  }
  return intersection_area / union_area;
}

double centerJumpRatio(const cv::Rect & a, const cv::Rect & b)
{
  const double ax = static_cast<double>(a.x) + 0.5 * static_cast<double>(a.width);
  const double ay = static_cast<double>(a.y) + 0.5 * static_cast<double>(a.height);
  const double bx = static_cast<double>(b.x) + 0.5 * static_cast<double>(b.width);
  const double by = static_cast<double>(b.y) + 0.5 * static_cast<double>(b.height);
  const double distance = std::hypot(ax - bx, ay - by);
  const double scale = std::max(
    1.0,
    std::hypot(
      static_cast<double>(std::max(a.width, b.width)),
      static_cast<double>(std::max(a.height, b.height))));
  return distance / scale;
}
}

CsrtIbvsNode::CsrtIbvsNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("csrt_ibvs_node", options)
{
  readParameters();

  const auto sensor_qos = rclcpp::SensorDataQoS();
  const auto default_qos = rclcpp::QoS(rclcpp::KeepLast(10));
  const auto init_bbox_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  const auto tracked_bbox_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic_, sensor_qos,
    [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) { onImage(msg); });

  if (use_depth_) {
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, sensor_qos,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) { onDepth(msg); });
  }

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, sensor_qos,
    [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { onCameraInfo(msg); });

  if (use_triangulation_after_min_depth_) {
    eef_bbox_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      eef_tracked_bbox_topic_, init_bbox_qos,
      [this](const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) { onEefBbox(msg); });

    eef_camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      eef_camera_info_topic_, sensor_qos,
      [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { onEefCameraInfo(msg); });
  }

  init_bbox_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    init_bbox_topic_, init_bbox_qos,
    [this](const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) { onInitBbox(msg); });

  base_hold_sub_ = create_subscription<std_msgs::msg::Bool>(
    base_hold_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    [this](const std_msgs::msg::Bool::ConstSharedPtr msg) { onBaseHold(msg); });

  tracked_bbox_pub_ =
    create_publisher<std_msgs::msg::Float32MultiArray>(tracked_bbox_topic_, tracked_bbox_qos);
  close_range_ready_pub_ =
    create_publisher<std_msgs::msg::Bool>(
    close_range_ready_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  if (enable_cmd_vel_) {
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, default_qos);
  }

  if (enable_arm_twist_) {
    arm_twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(arm_twist_topic_, default_qos);
  }

  if (publish_debug_image_) {
    debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, default_qos);
  }

  status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, status_qos);

  last_track_stamp_ = now();
  last_status_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  last_detector_bbox_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  latest_eef_bbox_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), [this]() { watchdog(); });

  RCLCPP_INFO(
    get_logger(),
    "hybrid_csrt_ibvs started: image=%s depth=%s camera_info=%s init_bbox=%s tracked_bbox=%s status=%s cmd_vel=%s eef_bbox=%s eef_camera_info=%s triangulation=%s",
    image_topic_.c_str(), depth_topic_.c_str(), camera_info_topic_.c_str(),
    init_bbox_topic_.c_str(), tracked_bbox_topic_.c_str(), status_topic_.c_str(),
    enable_cmd_vel_ ? cmd_vel_topic_.c_str() : "disabled",
    use_triangulation_after_min_depth_ ? eef_tracked_bbox_topic_.c_str() : "disabled",
    use_triangulation_after_min_depth_ ? eef_camera_info_topic_.c_str() : "disabled",
    use_triangulation_after_min_depth_ ? "enabled" : "disabled");

  publishStatus("ready; publish bbox [x, y, w, h] to " + init_bbox_topic_, true);
  publishCloseRangeReady(false, true);
}

void CsrtIbvsNode::readParameters()
{
  image_topic_ = declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
  depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
  camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");
  eef_tracked_bbox_topic_ = declare_parameter<std::string>("eef_tracked_bbox_topic", "/target/eef_tracked_bbox");
  eef_camera_info_topic_ = declare_parameter<std::string>("eef_camera_info_topic", "/eef_camera/camera_info");
  init_bbox_topic_ = declare_parameter<std::string>("init_bbox_topic", "/target/init_bbox");
  tracked_bbox_topic_ = declare_parameter<std::string>("tracked_bbox_topic", "/target/tracked_bbox");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  base_hold_topic_ = declare_parameter<std::string>("base_hold_topic", "/target/base_hold");
  close_range_ready_topic_ =
    declare_parameter<std::string>("close_range_ready_topic", "/target/close_range_ready");
  arm_twist_topic_ = declare_parameter<std::string>("arm_twist_topic", "/servo_node/delta_twist_cmds");
  arm_command_frame_id_ = declare_parameter<std::string>("arm_command_frame_id", "camera_color_optical_frame");
  debug_image_topic_ = declare_parameter<std::string>("debug_image_topic", "/hybrid_csrt_ibvs/debug_image");
  status_topic_ = declare_parameter<std::string>("status_topic", "/hybrid_csrt_ibvs/status");

  use_depth_ = declare_parameter<bool>("use_depth", true);
  use_area_fallback_ = declare_parameter<bool>("use_area_fallback", true);
  enable_cmd_vel_ = declare_parameter<bool>("enable_cmd_vel", true);
  enable_arm_twist_ = declare_parameter<bool>("enable_arm_twist", false);
  publish_debug_image_ = declare_parameter<bool>("publish_debug_image", true);
  stop_when_lost_ = declare_parameter<bool>("stop_when_lost", true);
  allow_reverse_ = declare_parameter<bool>("allow_reverse", true);
  reinitialize_while_tracking_ = declare_parameter<bool>("reinitialize_while_tracking", false);
  accept_detector_bbox_while_tracking_ = declare_parameter<bool>("accept_detector_bbox_while_tracking", true);
  lock_tracked_bbox_size_ = declare_parameter<bool>("lock_tracked_bbox_size", true);
  detector_bbox_override_timeout_s_ = declare_parameter<double>("detector_bbox_override_timeout_s", 0.75);
  min_tracked_bbox_size_ratio_ = declare_parameter<double>("min_tracked_bbox_size_ratio", 0.0);
  force_straight_approach_ = declare_parameter<bool>("force_straight_approach", false);
  enable_base_yaw_ = declare_parameter<bool>("enable_base_yaw", false);
  reinit_min_iou_ = declare_parameter<double>("reinit_min_iou", 0.02);
  reinit_max_center_jump_ratio_ = declare_parameter<double>("reinit_max_center_jump_ratio", 0.75);
  loss_frame_limit_ = declare_parameter<int>("loss_frame_limit", 8);
  min_bbox_size_px_ = declare_parameter<int>("min_bbox_size_px", 12);
  watchdog_timeout_s_ = declare_parameter<double>("watchdog_timeout_s", 0.7);
  status_period_s_ = declare_parameter<double>("status_period_s", 0.5);

  desired_u_ratio_ = declare_parameter<double>("desired_u_ratio", 0.5);
  desired_v_ratio_ = declare_parameter<double>("desired_v_ratio", 0.5);
  desired_depth_m_ = declare_parameter<double>("desired_depth_m", 0.45);
  desired_area_ratio_ = declare_parameter<double>("desired_area_ratio", 0.06);
  yaw_gain_ = declare_parameter<double>("yaw_gain", 1.35);
  linear_gain_ = declare_parameter<double>("linear_gain", 0.65);
  area_gain_ = declare_parameter<double>("area_gain", 1.2);
  arm_lateral_gain_ = declare_parameter<double>("arm_lateral_gain", 0.05);
  arm_vertical_gain_ = declare_parameter<double>("arm_vertical_gain", 0.05);
  arm_depth_gain_ = declare_parameter<double>("arm_depth_gain", 0.03);
  x_deadband_norm_ = declare_parameter<double>("x_deadband_norm", 0.035);
  y_deadband_norm_ = declare_parameter<double>("y_deadband_norm", 0.035);
  depth_deadband_m_ = declare_parameter<double>("depth_deadband_m", 0.035);
  area_deadband_ratio_ = declare_parameter<double>("area_deadband_ratio", 0.01);
  approach_yaw_gate_norm_ = declare_parameter<double>("approach_yaw_gate_norm", 0.28);
  straight_approach_depth_m_ = declare_parameter<double>("straight_approach_depth_m", 1.2);
  min_forward_approach_x_ = declare_parameter<double>("min_forward_approach_x", 0.0);
  max_linear_x_ = declare_parameter<double>("max_linear_x", 0.12);
  max_angular_z_ = declare_parameter<double>("max_angular_z", 0.55);
  max_arm_linear_ = declare_parameter<double>("max_arm_linear", 0.025);

  use_triangulation_after_min_depth_ =
    declare_parameter<bool>("use_triangulation_after_min_depth", false);
  use_eef_front_camera_extrinsic_override_ =
    declare_parameter<bool>("use_eef_front_camera_extrinsic_override", false);
  triangulation_start_depth_m_ =
    declare_parameter<double>("triangulation_start_depth_m", 0.47);
  triangulation_start_bbox_area_ratio_ =
    declare_parameter<double>("triangulation_start_bbox_area_ratio", 0.0);
  triangulation_start_bbox_height_ratio_ =
    declare_parameter<double>("triangulation_start_bbox_height_ratio", 0.0);
  triangulation_stop_x_m_ =
    declare_parameter<double>("triangulation_stop_x_m", 0.30);
  triangulation_max_speed_mps_ =
    declare_parameter<double>("triangulation_max_speed_mps", 0.04);
  triangulation_gain_ =
    declare_parameter<double>("triangulation_gain", 0.4);
  triangulation_fallback_speed_mps_ =
    declare_parameter<double>("triangulation_fallback_speed_mps", 0.005);
  triangulation_fallback_max_area_ratio_ =
    declare_parameter<double>("triangulation_fallback_max_area_ratio", 0.30);
  no_depth_visual_approach_speed_mps_ =
    declare_parameter<double>("no_depth_visual_approach_speed_mps", 0.0);
  no_depth_visual_approach_max_area_ratio_ =
    declare_parameter<double>("no_depth_visual_approach_max_area_ratio", 0.10);
  no_depth_visual_approach_max_height_ratio_ =
    declare_parameter<double>("no_depth_visual_approach_max_height_ratio", 0.60);
  triangulation_timeout_s_ =
    declare_parameter<double>("triangulation_timeout_s", 2.0);
  triangulation_min_range_m_ =
    declare_parameter<double>("triangulation_min_range_m", 0.06);
  triangulation_max_range_m_ =
    declare_parameter<double>("triangulation_max_range_m", 1.0);
  triangulation_max_ray_gap_m_ =
    declare_parameter<double>("triangulation_max_ray_gap_m", 0.08);
  eef_front_camera_offset_x_m_ =
    declare_parameter<double>("eef_front_camera_offset_x_m", -0.05);
  eef_front_camera_offset_y_m_ =
    declare_parameter<double>("eef_front_camera_offset_y_m", 0.0);
  eef_front_camera_offset_z_m_ =
    declare_parameter<double>("eef_front_camera_offset_z_m", 0.15);
  use_eef_front_camera_optical_offset_ =
    declare_parameter<bool>("use_eef_front_camera_optical_offset", false);
  eef_front_camera_optical_offset_x_m_ =
    declare_parameter<double>("eef_front_camera_optical_offset_x_m", 0.0);
  eef_front_camera_optical_offset_y_m_ =
    declare_parameter<double>("eef_front_camera_optical_offset_y_m", -0.15);
  eef_front_camera_optical_offset_z_m_ =
    declare_parameter<double>("eef_front_camera_optical_offset_z_m", -0.05);
  triangulation_flip_front_x_ =
    declare_parameter<bool>("triangulation_flip_front_x", false);
  triangulation_flip_front_y_ =
    declare_parameter<bool>("triangulation_flip_front_y", false);
  triangulation_flip_eef_x_ =
    declare_parameter<bool>("triangulation_flip_eef_x", false);
  triangulation_flip_eef_y_ =
    declare_parameter<bool>("triangulation_flip_eef_y", false);
  triangulation_flip_eef_z_ =
    declare_parameter<bool>("triangulation_flip_eef_z", false);

  depth_roi_radius_px_ = declare_parameter<int>("depth_roi_radius_px", 6);
  depth_bbox_inner_scale_ = declare_parameter<double>("depth_bbox_inner_scale", 0.7);
  depth_sample_percentile_ = declare_parameter<double>("depth_sample_percentile", 25.0);
  depth_min_valid_pixels_ = declare_parameter<int>("depth_min_valid_pixels", 5);
  depth_unit_scale_ = declare_parameter<double>("depth_unit_scale", 0.001);
  min_valid_depth_m_ = declare_parameter<double>("min_valid_depth_m", 0.12);
  max_valid_depth_m_ = declare_parameter<double>("max_valid_depth_m", 3.0);
  emergency_stop_depth_m_ = declare_parameter<double>("emergency_stop_depth_m", 0.18);
  max_depth_stamp_age_s_ = declare_parameter<double>("max_depth_stamp_age_s", 0.35);
  min_depth_samples_ = declare_parameter<int>("min_depth_samples", 30);
  stable_depth_frames_ = declare_parameter<int>("stable_depth_frames", 3);
  depth_std_max_m_ = declare_parameter<double>("depth_std_max_m", 0.08);
  depth_min_fill_ratio_ = declare_parameter<double>("depth_min_fill_ratio", 0.25);
  depth_jump_limit_m_ = declare_parameter<double>("depth_jump_limit_m", 0.10);

  loss_frame_limit_ = std::max(1, loss_frame_limit_);
  min_bbox_size_px_ = std::max(2, min_bbox_size_px_);
  depth_roi_radius_px_ = std::max(1, depth_roi_radius_px_);
  max_linear_x_ = std::max(0.0, max_linear_x_);
  max_angular_z_ = std::max(0.0, max_angular_z_);
  max_arm_linear_ = std::max(0.0, max_arm_linear_);
  triangulation_start_depth_m_ = std::max(0.0, triangulation_start_depth_m_);
  triangulation_start_bbox_area_ratio_ =
    clampValue(triangulation_start_bbox_area_ratio_, 0.0, 1.0);
  triangulation_start_bbox_height_ratio_ =
    clampValue(triangulation_start_bbox_height_ratio_, 0.0, 1.0);
  triangulation_stop_x_m_ = std::max(0.01, triangulation_stop_x_m_);
  triangulation_max_speed_mps_ = std::max(0.0, triangulation_max_speed_mps_);
  triangulation_gain_ = std::max(0.0, triangulation_gain_);
  triangulation_fallback_speed_mps_ =
    clampValue(triangulation_fallback_speed_mps_, 0.0, triangulation_max_speed_mps_);
  triangulation_fallback_max_area_ratio_ =
    clampValue(triangulation_fallback_max_area_ratio_, 0.0, 1.0);
  no_depth_visual_approach_speed_mps_ =
    clampValue(no_depth_visual_approach_speed_mps_, 0.0, max_linear_x_);
  no_depth_visual_approach_max_area_ratio_ =
    clampValue(no_depth_visual_approach_max_area_ratio_, 0.0, 1.0);
  no_depth_visual_approach_max_height_ratio_ =
    clampValue(no_depth_visual_approach_max_height_ratio_, 0.0, 1.0);
  triangulation_timeout_s_ = std::max(0.1, triangulation_timeout_s_);
  triangulation_min_range_m_ = std::max(0.0, triangulation_min_range_m_);
  triangulation_max_range_m_ = std::max(triangulation_min_range_m_ + 0.01, triangulation_max_range_m_);
  triangulation_max_ray_gap_m_ = std::max(0.001, triangulation_max_ray_gap_m_);
  straight_approach_depth_m_ = std::max(0.0, straight_approach_depth_m_);
  min_forward_approach_x_ = std::max(0.0, min_forward_approach_x_);
  reinit_min_iou_ = clampValue(reinit_min_iou_, 0.0, 1.0);
  reinit_max_center_jump_ratio_ = std::max(0.01, reinit_max_center_jump_ratio_);
  detector_bbox_override_timeout_s_ = std::max(0.0, detector_bbox_override_timeout_s_);
  min_tracked_bbox_size_ratio_ = clampValue(min_tracked_bbox_size_ratio_, 0.0, 1.0);
  depth_bbox_inner_scale_ = clampValue(depth_bbox_inner_scale_, 0.1, 1.0);
  depth_sample_percentile_ = clampValue(depth_sample_percentile_, 0.0, 100.0);
  depth_min_valid_pixels_ = std::max(1, depth_min_valid_pixels_);
  min_depth_samples_ = std::max(1, min_depth_samples_);
  stable_depth_frames_ = std::max(1, stable_depth_frames_);
  depth_std_max_m_ = std::max(0.0, depth_std_max_m_);
  depth_min_fill_ratio_ = clampValue(depth_min_fill_ratio_, 0.0, 1.0);
  depth_jump_limit_m_ = std::max(0.0, depth_jump_limit_m_);
}

void CsrtIbvsNode::onInitBbox(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg)
{
  if (msg->data.size() < 4) {
    RCLCPP_WARN(get_logger(), "init_bbox requires [x, y, w, h]. Received %zu values.", msg->data.size());
    return;
  }

  const auto x = static_cast<int>(std::lround(msg->data[0]));
  const auto y = static_cast<int>(std::lround(msg->data[1]));
  const auto w = static_cast<int>(std::lround(msg->data[2]));
  const auto h = static_cast<int>(std::lround(msg->data[3]));

  if (w <= 0 || h <= 0) {
    RCLCPP_WARN(get_logger(), "init_bbox width and height must be positive. Received [%d, %d, %d, %d].", x, y, w, h);
    return;
  }

  const cv::Rect new_bbox(x, y, w, h);
  const auto receive_time = now();
  std::string status_text;

  {
    std::lock_guard<std::mutex> lock(bbox_mutex_);
    if (state_ == TrackerState::TRACKING && hasTracker()) {
      if (last_tracked_bbox_) {
        const double iou = rectIou(*last_tracked_bbox_, new_bbox);
        const double jump = centerJumpRatio(*last_tracked_bbox_, new_bbox);
        if (iou < reinit_min_iou_ && jump > reinit_max_center_jump_ratio_) {
          std::ostringstream status;
          status << "tracking active; ignoring unrelated init bbox"
                 << " iou=" << std::fixed << std::setprecision(2) << iou
                 << " jump=" << std::fixed << std::setprecision(2) << jump;
          status_text = status.str();
        }
      }

      if (status_text.empty() && !reinitialize_while_tracking_) {
        if (accept_detector_bbox_while_tracking_) {
          detector_reference_bbox_ = new_bbox;
          last_detector_bbox_stamp_ = receive_time;
          status_text = "detector bbox accepted as primary bbox; CSRT remains backup";
        } else {
          status_text = "tracking active; ignoring new init bbox";
        }
      }
    }

    if (status_text.empty()) {
      pending_init_bbox_ = new_bbox;
      detector_reference_bbox_ = new_bbox;
      last_detector_bbox_stamp_ = receive_time;
      min_depth_reached_ = false;
      last_accepted_depth_m_.reset();
      pending_depth_m_.reset();
      stable_depth_count_ = 0;
      last_depth_filter_status_.clear();
      publishCloseRangeReady(false);
      status_text =
        state_ == TrackerState::TRACKING ?
        "bbox correction received; tracker will refresh on the next image" :
        "bbox received; tracker will initialize on the next image";
    }
  }

  publishStatus(status_text);
}

void CsrtIbvsNode::onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(camera_info_mutex_);
  fx_ = msg->k[0];
  fy_ = msg->k[4];
  camera_cx_ = msg->k[2];
  camera_cy_ = msg->k[5];
  have_camera_info_ = fx_ > 1.0 && fy_ > 1.0;
}

void CsrtIbvsNode::onEefBbox(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg)
{
  if (msg->data.size() < 4) {
    return;
  }

  const auto x = static_cast<int>(std::lround(msg->data[0]));
  const auto y = static_cast<int>(std::lround(msg->data[1]));
  const auto w = static_cast<int>(std::lround(msg->data[2]));
  const auto h = static_cast<int>(std::lround(msg->data[3]));
  if (w < min_bbox_size_px_ || h < min_bbox_size_px_) {
    return;
  }

  std::lock_guard<std::mutex> lock(eef_mutex_);
  latest_eef_bbox_ = cv::Rect(x, y, w, h);
  latest_eef_bbox_stamp_ = now();
}

void CsrtIbvsNode::onEefCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(eef_mutex_);
  eef_fx_ = msg->k[0];
  eef_fy_ = msg->k[4];
  eef_cx_ = msg->k[2];
  eef_cy_ = msg->k[5];
  have_eef_camera_info_ = eef_fx_ > 1.0 && eef_fy_ > 1.0;
}

void CsrtIbvsNode::onDepth(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  if (use_triangulation_after_min_depth_ && min_depth_reached_) {
    last_depth_filter_status_ = "depth ignored after 0.47m handoff";
    return;
  }

  try {
    const auto cv_ptr = cv_bridge::toCvShare(msg);
    DepthSample sample;
    sample.image = cv_ptr->image.clone();
    sample.encoding = msg->encoding;
    sample.stamp = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
    std::lock_guard<std::mutex> lock(depth_mutex_);
    latest_depth_ = std::move(sample);
  } catch (const cv_bridge::Exception & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1500, "depth cv_bridge conversion failed: %s", ex.what());
  }
}

void CsrtIbvsNode::onBaseHold(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  base_hold_active_.store(msg->data);
  if (msg->data) {
    publishStop(true);
    publishStatus("base hold active; publishing zero cmd_vel", true);
  } else {
    stop_sent_ = false;
    publishStatus("base hold released", true);
  }
}

void CsrtIbvsNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  const auto frame_bgr = imageMsgToBgr(msg);
  if (!frame_bgr) {
    return;
  }

  const cv::Mat & frame = *frame_bgr;
  if (frame.empty()) {
    publishStatus("empty image received");
    return;
  }

  rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  if (stamp.nanoseconds() == 0) {
    stamp = now();
  }

  std::optional<cv::Rect> bbox_for_init;
  {
    std::lock_guard<std::mutex> lock(bbox_mutex_);
    if (pending_init_bbox_) {
      bbox_for_init = sanitizeBox(*pending_init_bbox_, frame.size());
      pending_init_bbox_.reset();
    }
  }

  if (bbox_for_init) {
    initializeTracker(frame, *bbox_for_init);
  }

  if (state_ != TrackerState::TRACKING || !hasTracker()) {
    if (stop_when_lost_) {
      publishStop();
    }
    if (publish_debug_image_) {
      publishDebugImageNoTrack(
        msg,
        frame,
        state_ == TrackerState::LOST ?
        "target lost; waiting for fresh bbox" :
        "waiting for init bbox");
    }
    return;
  }

  cv::Rect tracked_box;
  bool tracking_ok = false;
  try {
    tracking_ok = updateTracker(frame, tracked_box);
  } catch (const cv::Exception & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1500, "%s update failed: %s",
      trackerBackendName(), ex.what());
    tracking_ok = false;
  }

  const auto clipped_box = sanitizeBox(tracked_box, frame.size());
  if (!tracking_ok || !clipped_box) {
    lost_count_ += 1;
    publishStatus("tracker uncertain; lost_count=" + std::to_string(lost_count_));

    if (publish_debug_image_) {
      if (clipped_box) {
        publishDebugImage(msg, frame, *clipped_box, IbvsResult(), false);
      } else {
        publishDebugImageNoTrack(msg, frame, "tracker uncertain; no valid bbox");
      }
    }

    if (lost_count_ >= loss_frame_limit_) {
      resetTracker();
      state_ = TrackerState::LOST;
      min_depth_reached_ = false;
      publishCloseRangeReady(false);
      if (stop_when_lost_) {
        publishStop(true);
      }
      publishStatus("target lost; publish a fresh bbox to reinitialize", true);
    }
    return;
  }

  lost_count_ = 0;
  last_track_stamp_ = now();
  state_ = TrackerState::TRACKING;
  const cv::Rect control_box = stabilizeTrackedBox(*clipped_box, frame.size(), now());
  {
    std::lock_guard<std::mutex> lock(bbox_mutex_);
    last_tracked_bbox_ = control_box;
  }

  auto ibvs = computeIbvsCommand(control_box, frame.size(), stamp);
  const bool base_hold = base_hold_active_.load();
  if (base_hold) {
    ibvs.base_cmd = geometry_msgs::msg::Twist();
  }

  if (enable_cmd_vel_ && cmd_vel_pub_) {
    cmd_vel_pub_->publish(ibvs.base_cmd);
    stop_sent_ = base_hold;
  }

  if (enable_arm_twist_ && arm_twist_pub_) {
    arm_twist_pub_->publish(ibvs.arm_cmd);
  }

  if (tracked_bbox_pub_) {
    std_msgs::msg::Float32MultiArray bbox_msg;
    bbox_msg.data = {
      static_cast<float>(control_box.x),
      static_cast<float>(control_box.y),
      static_cast<float>(control_box.width),
      static_cast<float>(control_box.height)};
    tracked_bbox_pub_->publish(bbox_msg);
  }

  if (publish_debug_image_) {
    publishDebugImage(msg, frame, control_box, ibvs, true);
  }

  std::ostringstream status;
	  status << "tracking bbox=[" << control_box.x << "," << control_box.y << ","
	         << control_box.width << "," << control_box.height << "]"
	         << " ex=" << ibvs.x_error_norm
	         << " ez=";
	  if (ibvs.depth_m) {
	    status << (*ibvs.depth_m - desired_depth_m_);
	  } else if (ibvs.triangulated_object_x_m) {
	    status << "triangulation_x=" << *ibvs.triangulated_object_x_m;
	  } else {
	    status << "no_depth_in_bbox";
	  }
	  status << " vx=" << ibvs.base_cmd.linear.x
	         << " wz=" << ibvs.base_cmd.angular.z
	         << " base_hold=" << (base_hold ? "true" : "false")
	         << " yaw_enabled=" << (enable_base_yaw_ ? "true" : "false");
	  if (!ibvs.range_status.empty()) {
	    status << " range_status=\"" << ibvs.range_status << "\"";
	  }
	  publishStatus(status.str());
	}

std::optional<cv::Mat> CsrtIbvsNode::imageMsgToBgr(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  const std::string & encoding = msg->encoding;
  if (encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      encoding == "16UC1" ||
      encoding == sensor_msgs::image_encodings::MONO16 ||
      encoding == "mono16" ||
      encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
      encoding == "32FC1") {
    return depthMsgToBgr(msg);
  }

  try {
    return cv_bridge::toCvCopy(msg, kBgr8)->image;
  } catch (const cv_bridge::Exception & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1500,
      "image cv_bridge conversion failed: %s", ex.what());
    return std::nullopt;
  }
}

std::optional<cv::Mat> CsrtIbvsNode::depthMsgToBgr(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv::Mat depth_m;

  if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      msg->encoding == "16UC1" ||
      msg->encoding == sensor_msgs::image_encodings::MONO16 ||
      msg->encoding == "mono16") {
    try {
      const auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      cv_ptr->image.convertTo(depth_m, CV_32FC1, depth_unit_scale_);
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1500,
        "depth image conversion failed: %s", ex.what());
      return std::nullopt;
    }
  } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
             msg->encoding == "32FC1") {
    try {
      depth_m = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1500,
        "depth image conversion failed: %s", ex.what());
      return std::nullopt;
    }
  } else {
    return std::nullopt;
  }

  if (depth_m.empty()) {
    return std::nullopt;
  }

  cv::Mat gray(depth_m.size(), CV_8UC1, cv::Scalar(0));
  const double range = std::max(0.001, max_valid_depth_m_ - min_valid_depth_m_);
  for (int row = 0; row < depth_m.rows; ++row) {
    const float * depth_row = depth_m.ptr<float>(row);
    uint8_t * gray_row = gray.ptr<uint8_t>(row);
    for (int col = 0; col < depth_m.cols; ++col) {
      const double z = depth_row[col];
      if (!std::isfinite(z) || z < min_valid_depth_m_ || z > max_valid_depth_m_) {
        continue;
      }
      const double normalized = (max_valid_depth_m_ - z) / range;
      gray_row[col] = static_cast<uint8_t>(clampValue(normalized, 0.0, 1.0) * 255.0);
    }
  }

  cv::Mat bgr;
  cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
  return bgr;
}

bool CsrtIbvsNode::initializeTracker(const cv::Mat & frame, const cv::Rect & bbox)
{
  try {
#ifdef HYBRID_CSRT_IBVS_HAS_OPENCV_TRACKING
    tracker_ = cv::TrackerCSRT::create();
    tracker_->init(frame, bbox);
#else
    cv::Mat hsv;
    cv::Mat mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 30, 20), cv::Scalar(180, 255, 255), mask);

    const cv::Mat roi_mask = mask(bbox);
    const int color_pixels = cv::countNonZero(roi_mask);
    const int min_color_pixels = std::max(8, bbox.area() / 100);
    if (color_pixels >= min_color_pixels) {
      const cv::Mat roi = hsv(bbox);
      const int channels[] = {0, 1};
      const int hist_size[] = {30, 32};
      const float h_range[] = {0.0F, 180.0F};
      const float s_range[] = {0.0F, 256.0F};
      const float * ranges[] = {h_range, s_range};

      cv::calcHist(&roi, 1, channels, roi_mask, tracker_hist_, 2, hist_size, ranges);
      tracker_uses_gray_ = false;
    } else {
      cv::Mat gray;
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
      const cv::Mat roi = gray(bbox);
      const int channels[] = {0};
      const int hist_size[] = {32};
      const float gray_range[] = {1.0F, 256.0F};
      const float * ranges[] = {gray_range};

      cv::calcHist(&roi, 1, channels, cv::Mat(), tracker_hist_, 1, hist_size, ranges);
      tracker_uses_gray_ = true;
    }
    if (cv::countNonZero(tracker_hist_) == 0) {
      throw cv::Exception(cv::Error::StsBadArg, "empty CamShift histogram", __func__, __FILE__, __LINE__);
    }
    cv::normalize(tracker_hist_, tracker_hist_, 0.0, 255.0, cv::NORM_MINMAX);
    tracker_window_ = bbox;
    tracker_initialized_ = true;
#endif
    lost_count_ = 0;
    state_ = TrackerState::TRACKING;
    last_track_stamp_ = now();
    {
      std::lock_guard<std::mutex> lock(bbox_mutex_);
      last_tracked_bbox_ = bbox;
      detector_reference_bbox_ = bbox;
      last_detector_bbox_stamp_ = now();
    }
    publishStatus(std::string(trackerBackendName()) + " initialized", true);
    return true;
  } catch (const cv::Exception & ex) {
    resetTracker();
    state_ = TrackerState::LOST;
    min_depth_reached_ = false;
    publishCloseRangeReady(false);
    publishStop(true);
    RCLCPP_ERROR(get_logger(), "%s initialization failed: %s", trackerBackendName(), ex.what());
    publishStatus(std::string(trackerBackendName()) + " initialization failed", true);
    return false;
  }
}

bool CsrtIbvsNode::updateTracker(const cv::Mat & frame, cv::Rect & bbox)
{
#ifdef HYBRID_CSRT_IBVS_HAS_OPENCV_TRACKING
  return tracker_ && tracker_->update(frame, bbox);
#else
  if (!tracker_initialized_ || tracker_hist_.empty() || tracker_window_.empty()) {
    return false;
  }

  cv::Mat hsv;
  cv::Mat mask;
  cv::Mat back_project;
  if (tracker_uses_gray_) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    const int channels[] = {0};
    const float gray_range[] = {1.0F, 256.0F};
    const float * ranges[] = {gray_range};
    cv::calcBackProject(&gray, 1, channels, tracker_hist_, back_project, ranges);
  } else {
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 30, 20), cv::Scalar(180, 255, 255), mask);

    const int channels[] = {0, 1};
    const float h_range[] = {0.0F, 180.0F};
    const float s_range[] = {0.0F, 256.0F};
    const float * ranges[] = {h_range, s_range};
    cv::calcBackProject(&hsv, 1, channels, tracker_hist_, back_project, ranges);
    cv::bitwise_and(back_project, mask, back_project);
  }

  if (back_project.empty() || cv::sum(back_project)[0] <= 0.0) {
    return false;
  }

  const cv::Rect image_bounds(0, 0, frame.cols, frame.rows);
  tracker_window_ = tracker_window_ & image_bounds;
  if (tracker_window_.empty()) {
    return false;
  }

  const auto criteria = cv::TermCriteria(
    cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 10, 1.0);
  const cv::RotatedRect tracked = cv::CamShift(back_project, tracker_window_, criteria);
  const cv::Rect tracked_box = tracked.boundingRect() & image_bounds;
  if (tracked_box.empty()) {
    return false;
  }

  bbox = tracked_box;
  return true;
#endif
}

bool CsrtIbvsNode::hasTracker() const
{
#ifdef HYBRID_CSRT_IBVS_HAS_OPENCV_TRACKING
  return static_cast<bool>(tracker_);
#else
  return tracker_initialized_;
#endif
}

void CsrtIbvsNode::resetTracker()
{
#ifdef HYBRID_CSRT_IBVS_HAS_OPENCV_TRACKING
  tracker_.release();
#else
  tracker_hist_.release();
  tracker_window_ = cv::Rect();
  tracker_initialized_ = false;
  tracker_uses_gray_ = false;
#endif
  std::lock_guard<std::mutex> lock(bbox_mutex_);
  pending_init_bbox_.reset();
  last_tracked_bbox_.reset();
  detector_reference_bbox_.reset();
  last_detector_bbox_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
}

const char * CsrtIbvsNode::trackerBackendName() const
{
#ifdef HYBRID_CSRT_IBVS_HAS_OPENCV_TRACKING
  return "CSRT";
#else
  return "CamShift fallback";
#endif
}

std::optional<cv::Rect> CsrtIbvsNode::sanitizeBox(const cv::Rect & bbox, const cv::Size & image_size) const
{
  if (bbox.width <= 0 || bbox.height <= 0 || image_size.width <= 0 || image_size.height <= 0) {
    return std::nullopt;
  }

  const cv::Rect image_bounds(0, 0, image_size.width, image_size.height);
  const cv::Rect clipped = bbox & image_bounds;

  if (clipped.width < min_bbox_size_px_ || clipped.height < min_bbox_size_px_) {
    return std::nullopt;
  }

  return clipped;
}

cv::Rect CsrtIbvsNode::stabilizeTrackedBox(
  const cv::Rect & tracker_bbox,
  const cv::Size & image_size,
  const rclcpp::Time & current_time) const
{
  std::optional<cv::Rect> detector_reference;
  rclcpp::Time detector_stamp(0, 0, get_clock()->get_clock_type());
  {
    std::lock_guard<std::mutex> lock(bbox_mutex_);
    detector_reference = detector_reference_bbox_;
    detector_stamp = last_detector_bbox_stamp_;
  }

  if (!detector_reference) {
    return tracker_bbox;
  }

  if (
    detector_bbox_override_timeout_s_ > 0.0 &&
    detector_stamp.nanoseconds() != 0 &&
    (current_time - detector_stamp).seconds() <= detector_bbox_override_timeout_s_)
  {
    const auto detector_box = sanitizeBox(*detector_reference, image_size);
    if (detector_box) {
      return *detector_box;
    }
  }

  int width = tracker_bbox.width;
  int height = tracker_bbox.height;

  if (lock_tracked_bbox_size_) {
    width = detector_reference->width;
    height = detector_reference->height;
  } else if (min_tracked_bbox_size_ratio_ > 0.0) {
    const int min_width = static_cast<int>(std::lround(
      static_cast<double>(detector_reference->width) * min_tracked_bbox_size_ratio_));
    const int min_height = static_cast<int>(std::lround(
      static_cast<double>(detector_reference->height) * min_tracked_bbox_size_ratio_));
    width = std::max(width, min_width);
    height = std::max(height, min_height);
  } else {
    return tracker_bbox;
  }

  width = clampValue(width, min_bbox_size_px_, image_size.width);
  height = clampValue(height, min_bbox_size_px_, image_size.height);
  const double center_x =
    static_cast<double>(tracker_bbox.x) + 0.5 * static_cast<double>(tracker_bbox.width);
  const double center_y =
    static_cast<double>(tracker_bbox.y) + 0.5 * static_cast<double>(tracker_bbox.height);

  const cv::Rect detector_sized_box(
    static_cast<int>(std::lround(center_x - 0.5 * static_cast<double>(width))),
    static_cast<int>(std::lround(center_y - 0.5 * static_cast<double>(height))),
    width,
    height);

  const auto clipped = sanitizeBox(detector_sized_box, image_size);
  return clipped.value_or(tracker_bbox);
}

bool CsrtIbvsNode::isEefBboxFresh()
{
  std::lock_guard<std::mutex> lock(eef_mutex_);
  if (!latest_eef_bbox_ || latest_eef_bbox_stamp_.nanoseconds() == 0) {
    return false;
  }
  return (get_clock()->now() - latest_eef_bbox_stamp_).seconds() <= triangulation_timeout_s_;
}

std::optional<double> CsrtIbvsNode::estimateTriangulatedObjectX(
  const cv::Rect & front_bbox,
  const cv::Size & front_image_size,
  std::string * reason)
{
  static_cast<void>(front_image_size);
  const auto set_reason = [reason](const std::string & text) {
    if (reason) {
      *reason = text;
    }
  };

  if (!use_eef_front_camera_extrinsic_override_) {
    set_reason("EEF-front extrinsic override disabled");
    return std::nullopt;
  }

  bool have_front_info = false;
  double front_fx = 0.0;
  double front_fy = 0.0;
  double front_cx = 0.0;
  double front_cy = 0.0;
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    have_front_info = have_camera_info_;
    front_fx = fx_;
    front_fy = fy_;
    front_cx = camera_cx_;
    front_cy = camera_cy_;
  }
  if (!have_front_info || front_fx <= 1.0 || front_fy <= 1.0) {
    set_reason("missing front camera info");
    return std::nullopt;
  }

  cv::Rect eef_bbox;
  double eef_fx = 0.0;
  double eef_fy = 0.0;
  double eef_cx = 0.0;
  double eef_cy = 0.0;
  {
    std::lock_guard<std::mutex> lock(eef_mutex_);
    if (!latest_eef_bbox_) {
      set_reason("missing eef bbox");
      return std::nullopt;
    }
    if (latest_eef_bbox_stamp_.nanoseconds() == 0 ||
        (get_clock()->now() - latest_eef_bbox_stamp_).seconds() > triangulation_timeout_s_)
    {
      set_reason("stale eef bbox");
      return std::nullopt;
    }
    if (!have_eef_camera_info_ || eef_fx_ <= 1.0 || eef_fy_ <= 1.0) {
      set_reason("missing eef camera info");
      return std::nullopt;
    }
    eef_bbox = *latest_eef_bbox_;
    eef_fx = eef_fx_;
    eef_fy = eef_fy_;
    eef_cx = eef_cx_;
    eef_cy = eef_cy_;
  }

  const auto normalize = [](const cv::Point3d & ray) -> std::optional<cv::Point3d> {
    const double length = std::sqrt(ray.dot(ray));
    if (!std::isfinite(length) || length < 1e-9) {
      return std::nullopt;
    }
    return cv::Point3d(ray.x / length, ray.y / length, ray.z / length);
  };

  const double front_u = static_cast<double>(front_bbox.x) + 0.5 * static_cast<double>(front_bbox.width);
  const double front_v = static_cast<double>(front_bbox.y) + 0.5 * static_cast<double>(front_bbox.height);
  const double eef_u = static_cast<double>(eef_bbox.x) + 0.5 * static_cast<double>(eef_bbox.width);
  const double eef_v = static_cast<double>(eef_bbox.y) + 0.5 * static_cast<double>(eef_bbox.height);

  double front_ray_x = (front_u - front_cx) / front_fx;
  double front_ray_y = (front_v - front_cy) / front_fy;
  double front_ray_z = 1.0;
  if (triangulation_flip_front_x_) {
    front_ray_x = -front_ray_x;
  }
  if (triangulation_flip_front_y_) {
    front_ray_y = -front_ray_y;
  }

  double eef_ray_x = (eef_u - eef_cx) / eef_fx;
  double eef_ray_y = (eef_v - eef_cy) / eef_fy;
  double eef_ray_z = 1.0;
  if (triangulation_flip_eef_x_) {
    eef_ray_x = -eef_ray_x;
  }
  if (triangulation_flip_eef_y_) {
    eef_ray_y = -eef_ray_y;
  }
  if (triangulation_flip_eef_z_) {
    eef_ray_z = -eef_ray_z;
  }

  auto front_direction = normalize(cv::Point3d(front_ray_x, front_ray_y, front_ray_z));
  auto eef_direction = normalize(cv::Point3d(eef_ray_x, eef_ray_y, eef_ray_z));
  if (!front_direction || !eef_direction) {
    set_reason("invalid triangulation ray");
    return std::nullopt;
  }

  const cv::Point3d front_origin(0.0, 0.0, 0.0);
  cv::Point3d eef_origin;
  if (use_eef_front_camera_optical_offset_) {
    eef_origin = cv::Point3d(
      eef_front_camera_optical_offset_x_m_,
      eef_front_camera_optical_offset_y_m_,
      eef_front_camera_optical_offset_z_m_);
  } else {
    // Config offsets are measured in base_link axes: x forward, y left, z up.
    // Camera optical axes are x right, y down, z forward.
    eef_origin = cv::Point3d(
      -eef_front_camera_offset_y_m_,
      -eef_front_camera_offset_z_m_,
      eef_front_camera_offset_x_m_);
  }

  const cv::Point3d w0 = front_origin - eef_origin;
  const double a = front_direction->dot(*front_direction);
  const double b = front_direction->dot(*eef_direction);
  const double c = eef_direction->dot(*eef_direction);
  const double d = front_direction->dot(w0);
  const double e = eef_direction->dot(w0);
  const double denom = a * c - b * b;
  if (std::abs(denom) < 1e-6) {
    set_reason("camera rays nearly parallel");
    return std::nullopt;
  }

  const double front_range = (b * e - c * d) / denom;
  const double eef_range = (a * e - b * d) / denom;
  if (front_range < triangulation_min_range_m_ || front_range > triangulation_max_range_m_ ||
      eef_range < triangulation_min_range_m_ || eef_range > triangulation_max_range_m_) {
    std::ostringstream status;
    status << "triangulation range invalid: front_range=" << front_range
           << " eef_range=" << eef_range
           << " front_uv=(" << front_u << ", " << front_v << ")"
           << " eef_uv=(" << eef_u << ", " << eef_v << ")"
           << " front_k=(" << front_fx << ", " << front_fy << ", "
           << front_cx << ", " << front_cy << ")"
           << " eef_k=(" << eef_fx << ", " << eef_fy << ", "
           << eef_cx << ", " << eef_cy << ")"
           << " front_ray=(" << front_direction->x << ", "
           << front_direction->y << ", " << front_direction->z << ")"
           << " eef_ray=(" << eef_direction->x << ", "
           << eef_direction->y << ", " << eef_direction->z << ")"
           << " eef_origin=(" << eef_origin.x << ", "
           << eef_origin.y << ", " << eef_origin.z << ")"
           << " denom=" << denom
           << " optical_offset="
           << (use_eef_front_camera_optical_offset_ ? "true" : "false");
    set_reason(status.str());
    return std::nullopt;
  }

  const cv::Point3d front_point = front_origin + (*front_direction * front_range);
  const cv::Point3d eef_point = eef_origin + (*eef_direction * eef_range);
  const cv::Point3d gap = front_point - eef_point;
  const double ray_gap = std::sqrt(gap.dot(gap));
  if (!std::isfinite(ray_gap) || ray_gap > triangulation_max_ray_gap_m_) {
    std::ostringstream status;
    status << "ray gap too large " << ray_gap;
    set_reason(status.str());
    return std::nullopt;
  }

  const cv::Point3d object = (front_point + eef_point) * 0.5;
  const double object_x = object.z;
  if (!std::isfinite(object_x) ||
      object_x < triangulation_min_range_m_ ||
      object_x > triangulation_max_range_m_) {
    std::ostringstream status;
    status << "triangulation object_x out of range " << object_x;
    set_reason(status.str());
    return std::nullopt;
  }

  if (reason) {
    std::ostringstream status;
    status << "object_x=" << object_x << " ray_gap=" << ray_gap;
    *reason = status.str();
  }
  return object_x;
}

CsrtIbvsNode::IbvsResult CsrtIbvsNode::computeIbvsCommand(
  const cv::Rect & bbox, const cv::Size & image_size, const rclcpp::Time & stamp)
{
  IbvsResult result;
  result.arm_cmd.header.stamp = stamp;
  result.arm_cmd.header.frame_id = arm_command_frame_id_;

  double linear_x = 0.0;
  double angular_z = 0.0;

  const double target_u = static_cast<double>(bbox.x) + 0.5 * static_cast<double>(bbox.width);
  const double target_v = static_cast<double>(bbox.y) + 0.5 * static_cast<double>(bbox.height);
  const double desired_u = desired_u_ratio_ * static_cast<double>(image_size.width);
  const double desired_v = desired_v_ratio_ * static_cast<double>(image_size.height);
  result.area_ratio = static_cast<double>(bbox.width * bbox.height) /
    static_cast<double>(std::max(1, image_size.width * image_size.height));

  const double bbox_height_ratio =
    static_cast<double>(bbox.height) / static_cast<double>(image_size.height);
  const double bbox_area_ratio =
    static_cast<double>(bbox.width * bbox.height) /
    static_cast<double>(image_size.width * image_size.height);

  if (use_triangulation_after_min_depth_ && min_depth_reached_) {
    if (bbox_height_ratio >= 0.70 || bbox_area_ratio >= 0.30) {
      linear_x = 0.0;
      angular_z = 0.0;
      publishCloseRangeReady(true);

      std::ostringstream status;
      status << "close range ready by front bbox scale"
            << " height_ratio=" << bbox_height_ratio
            << " area_ratio=" << bbox_area_ratio;
      result.range_status = status.str();
      return result;
    }
  }

  bool have_info = false;
  double fx = 0.0;
  double fy = 0.0;
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    have_info = have_camera_info_;
    fx = fx_;
    fy = fy_;
  }

  if (have_info && fx > 1.0 && fy > 1.0) {
    result.x_error_norm = (target_u - desired_u) / fx;
    result.y_error_norm = (target_v - desired_v) / fy;
  } else {
    result.x_error_norm = (target_u - desired_u) / static_cast<double>(std::max(1, image_size.width));
    result.y_error_norm = (target_v - desired_v) / static_cast<double>(std::max(1, image_size.height));
  }

  if (std::abs(result.x_error_norm) > x_deadband_norm_) {
    // Match the TurtleBot3 manipulation base command convention: positive
    // image x error should rotate toward the target in the real robot frame.
    angular_z = yaw_gain_ * result.x_error_norm;
  }

  result.depth_m = estimateDepthMeters(bbox, image_size, stamp);
  const bool straight_depth_approach =
    force_straight_approach_ &&
    (!result.depth_m || straight_approach_depth_m_ <= 0.0 || *result.depth_m <= straight_approach_depth_m_);
  const bool visual_triangulation_start =
    use_triangulation_after_min_depth_ &&
    !result.depth_m &&
    ((triangulation_start_bbox_area_ratio_ > 0.0 &&
      bbox_area_ratio >= triangulation_start_bbox_area_ratio_) ||
    (triangulation_start_bbox_height_ratio_ > 0.0 &&
      bbox_height_ratio >= triangulation_start_bbox_height_ratio_));

  if (result.depth_m && *result.depth_m < emergency_stop_depth_m_) {
    result.depth_too_close = true;
    linear_x = 0.0;
    angular_z = 0.0;
    publishCloseRangeReady(false);
    std::ostringstream status;
    status << "depth emergency stop: depth=" << *result.depth_m
           << " emergency=" << emergency_stop_depth_m_;
    result.range_status = status.str();
  } else {
    const bool reached_triangulation_start =
      use_triangulation_after_min_depth_ &&
      result.depth_m &&
      *result.depth_m <= triangulation_start_depth_m_ + depth_deadband_m_;
    if (reached_triangulation_start || visual_triangulation_start) {
      min_depth_reached_ = true;
    }
  }

  if (!result.depth_too_close &&
      use_triangulation_after_min_depth_ &&
      min_depth_reached_ &&
      (bbox_height_ratio >= 0.70 || bbox_area_ratio >= 0.30))
  {
    linear_x = 0.0;
    angular_z = 0.0;
    publishCloseRangeReady(true);

    std::ostringstream status;
    status << "close range ready by front bbox scale"
           << " height_ratio=" << bbox_height_ratio
           << " area_ratio=" << bbox_area_ratio;
    result.range_status = status.str();
    return result;
  }

  bool no_depth_visual_approach_active = false;

  if (!result.depth_too_close &&
      use_triangulation_after_min_depth_ &&
      min_depth_reached_)
  {
    result.using_triangulation = true;
    std::string tri_reason;
    auto triangulated_x = estimateTriangulatedObjectX(bbox, image_size, &tri_reason);
    if (!triangulated_x) {
      const double area_error = desired_area_ratio_ - result.area_ratio;
      const bool allow_visual_handoff_fallback =
        visual_triangulation_start &&
        triangulation_fallback_max_area_ratio_ > 0.0 &&
        result.area_ratio < triangulation_fallback_max_area_ratio_;
      if (triangulation_fallback_speed_mps_ > 0.0 &&
          (area_error > area_deadband_ratio_ || allow_visual_handoff_fallback)) {
        linear_x = area_error > area_deadband_ratio_ ?
          clampValue(area_gain_ * area_error, 0.0, triangulation_fallback_speed_mps_) :
          triangulation_fallback_speed_mps_;
        publishCloseRangeReady(false);
        std::ostringstream status;
        status << "triangulation fallback approach: " << tri_reason
               << " area=" << result.area_ratio
               << " target_area=" << desired_area_ratio_
               << " visual_handoff=" << (allow_visual_handoff_fallback ? "true" : "false")
               << " height_ratio=" << bbox_height_ratio
               << " vx=" << linear_x;
        result.range_status = status.str();
      } else {
        linear_x = 0.0;
        angular_z = 0.0;
        publishCloseRangeReady(false);
        result.range_status = "triangulation waiting: " + tri_reason;
      }
    } else {
      result.triangulated_object_x_m = triangulated_x;
      const double error_x = *triangulated_x - triangulation_stop_x_m_;
      if (error_x <= 0.0) {
        linear_x = 0.0;
        angular_z = 0.0;
        publishCloseRangeReady(true);
        std::ostringstream status;
        status << "triangulation stop reached: object_x=" << *triangulated_x
               << " target=" << triangulation_stop_x_m_;
        result.range_status = status.str();
      } else {
        linear_x = clampValue(
          triangulation_gain_ * error_x,
          0.0,
          triangulation_max_speed_mps_);
        publishCloseRangeReady(false);
        std::ostringstream status;
        status << "triangulation approach: object_x=" << *triangulated_x
               << " target=" << triangulation_stop_x_m_
               << " vx=" << linear_x
               << " " << tri_reason;
        result.range_status = status.str();
      }
    }
  } else if (!result.depth_too_close && result.depth_m) {
    publishCloseRangeReady(false);
    const double depth_error = *result.depth_m - desired_depth_m_;
    if (std::abs(depth_error) > depth_deadband_m_) {
      linear_x = linear_gain_ * depth_error;
    }
    std::ostringstream status;
    status << "depth approach: depth=" << *result.depth_m
           << " target=" << desired_depth_m_
           << " vx=" << linear_x;
    result.range_status = status.str();
  } else if (
      !result.depth_too_close &&
      use_triangulation_after_min_depth_ &&
      !min_depth_reached_ &&
      !result.depth_m &&
      no_depth_visual_approach_speed_mps_ > 0.0 &&
      (no_depth_visual_approach_max_area_ratio_ <= 0.0 ||
      bbox_area_ratio < no_depth_visual_approach_max_area_ratio_) &&
      (no_depth_visual_approach_max_height_ratio_ <= 0.0 ||
      bbox_height_ratio < no_depth_visual_approach_max_height_ratio_))
  {
    publishCloseRangeReady(false);
    linear_x = no_depth_visual_approach_speed_mps_;
    no_depth_visual_approach_active = true;
    std::ostringstream status;
    status << "no-depth visual approach before triangulation handoff"
           << " area=" << result.area_ratio
           << " start_area=" << triangulation_start_bbox_area_ratio_
           << " height_ratio=" << bbox_height_ratio
           << " start_height=" << triangulation_start_bbox_height_ratio_
           << " vx=" << linear_x;
    result.range_status = status.str();
  } else if (use_area_fallback_) {
    publishCloseRangeReady(false);
    const double area_error = desired_area_ratio_ - result.area_ratio;
    if (std::abs(area_error) > area_deadband_ratio_) {
      linear_x = area_gain_ * area_error;
    }
    std::ostringstream status;
    status << "area fallback: area=" << result.area_ratio
           << " target=" << desired_area_ratio_
           << " vx=" << linear_x;
    result.range_status = status.str();
  } else {
    linear_x = 0.0;
    angular_z = 0.0;
    publishCloseRangeReady(false);
    result.range_status = "no valid depth; area fallback disabled";
  }

  if (straight_depth_approach) {
    angular_z = 0.0;
    if (linear_x > 0.0 && min_forward_approach_x_ > 0.0) {
      linear_x = std::max(linear_x, min_forward_approach_x_);
    }
  } else if (linear_x > 0.0 && approach_yaw_gate_norm_ > 1e-6 && !no_depth_visual_approach_active) {
    const double gate = clampValue(1.0 - std::abs(result.x_error_norm) / approach_yaw_gate_norm_, 0.0, 1.0);
    linear_x *= gate;
  }

  if (!allow_reverse_ && linear_x < 0.0) {
    linear_x = 0.0;
  }
  if (!enable_base_yaw_) {
    angular_z = 0.0;
  }

  result.base_cmd.linear.x = clampValue(linear_x, allow_reverse_ ? -max_linear_x_ : 0.0, max_linear_x_);
  result.base_cmd.angular.z = clampValue(angular_z, -max_angular_z_, max_angular_z_);

  if (enable_arm_twist_) {
    double arm_y = 0.0;
    double arm_z = 0.0;
    double arm_x = 0.0;

    if (std::abs(result.x_error_norm) > x_deadband_norm_) {
      arm_y = -arm_lateral_gain_ * result.x_error_norm;
    }
    if (std::abs(result.y_error_norm) > y_deadband_norm_) {
      arm_z = -arm_vertical_gain_ * result.y_error_norm;
    }
    if (result.depth_m) {
      const double depth_error = *result.depth_m - desired_depth_m_;
      if (std::abs(depth_error) > depth_deadband_m_) {
        arm_x = arm_depth_gain_ * depth_error;
      }
    }

    result.arm_cmd.twist.linear.x = clampValue(arm_x, -max_arm_linear_, max_arm_linear_);
    result.arm_cmd.twist.linear.y = clampValue(arm_y, -max_arm_linear_, max_arm_linear_);
    result.arm_cmd.twist.linear.z = clampValue(arm_z, -max_arm_linear_, max_arm_linear_);
  }

  return result;
}

std::optional<double> CsrtIbvsNode::estimateDepthMeters(
  const cv::Rect & bbox, const cv::Size & image_size, const rclcpp::Time & image_stamp)
{
  if (!use_depth_) {
    last_depth_filter_status_ = "depth disabled by parameter";
    return std::nullopt;
  }

  if (use_triangulation_after_min_depth_ && min_depth_reached_) {
    last_depth_filter_status_ = "depth disabled after 0.47m handoff";
    return std::nullopt;
  }

  std::optional<DepthSample> sample;
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    sample = latest_depth_;
  }

  if (!sample || sample->image.empty()) {
    last_depth_filter_status_ = "depth missing";
    return std::nullopt;
  }

  if (max_depth_stamp_age_s_ > 0.0 && image_stamp.nanoseconds() != 0 && sample->stamp.nanoseconds() != 0) {
    const double age_s = std::abs((image_stamp - sample->stamp).seconds());
    if (age_s > max_depth_stamp_age_s_) {
      std::ostringstream status;
      status << "depth rejected: stale age=" << age_s
             << "s > " << max_depth_stamp_age_s_ << "s";
      last_depth_filter_status_ = status.str();
      return std::nullopt;
    }
  }

  const cv::Rect depth_bounds(0, 0, sample->image.cols, sample->image.rows);
  const double scale_x =
    static_cast<double>(sample->image.cols) / static_cast<double>(std::max(1, image_size.width));
  const double scale_y =
    static_cast<double>(sample->image.rows) / static_cast<double>(std::max(1, image_size.height));
  const double center_x = (static_cast<double>(bbox.x) + 0.5 * static_cast<double>(bbox.width)) * scale_x;
  const double center_y = (static_cast<double>(bbox.y) + 0.5 * static_cast<double>(bbox.height)) * scale_y;
  const double roi_width = std::max(1.0, static_cast<double>(bbox.width) * scale_x * depth_bbox_inner_scale_);
  const double roi_height = std::max(1.0, static_cast<double>(bbox.height) * scale_y * depth_bbox_inner_scale_);
  const cv::Rect roi(
    static_cast<int>(std::floor(center_x - 0.5 * roi_width)),
    static_cast<int>(std::floor(center_y - 0.5 * roi_height)),
    static_cast<int>(std::ceil(roi_width)),
    static_cast<int>(std::ceil(roi_height)));
  const cv::Rect clipped_roi = roi & depth_bounds;
  if (clipped_roi.empty()) {
    last_depth_filter_status_ = "depth rejected: empty bbox ROI";
    return std::nullopt;
  }

  std::vector<double> valid_depths;
  valid_depths.reserve(static_cast<size_t>(clipped_roi.width * clipped_roi.height));
  const int step = std::max(1, std::min(clipped_roi.width, clipped_roi.height) / 32);
  int sampled_pixels = 0;

  for (int r = clipped_roi.y; r < clipped_roi.y + clipped_roi.height; r += step) {
    for (int c = clipped_roi.x; c < clipped_roi.x + clipped_roi.width; c += step) {
      ++sampled_pixels;
      const auto meters = pixelToMeters(sample->image, sample->encoding, r, c);
      if (meters) {
        valid_depths.push_back(*meters);
      }
    }
  }

  const int required_samples = std::max(depth_min_valid_pixels_, min_depth_samples_);
  if (static_cast<int>(valid_depths.size()) < required_samples) {
    std::ostringstream status;
    status << "depth rejected: samples=" << valid_depths.size()
           << " < " << required_samples;
    last_depth_filter_status_ = status.str();
    return std::nullopt;
  }

  const double fill_ratio = sampled_pixels > 0 ?
    static_cast<double>(valid_depths.size()) / static_cast<double>(sampled_pixels) : 0.0;
  if (fill_ratio < depth_min_fill_ratio_) {
    std::ostringstream status;
    status << "depth rejected: fill=" << fill_ratio
           << " < " << depth_min_fill_ratio_;
    last_depth_filter_status_ = status.str();
    return std::nullopt;
  }

  double mean = 0.0;
  for (const double depth : valid_depths) {
    mean += depth;
  }
  mean /= static_cast<double>(valid_depths.size());

  double variance = 0.0;
  for (const double depth : valid_depths) {
    const double diff = depth - mean;
    variance += diff * diff;
  }
  variance /= static_cast<double>(valid_depths.size());
  const double std_m = std::sqrt(variance);

  if (std_m > depth_std_max_m_) {
    std::ostringstream status;
    status << "depth rejected: std=" << std_m
           << "m > " << depth_std_max_m_ << "m";
    last_depth_filter_status_ = status.str();
    return std::nullopt;
  }

  std::sort(valid_depths.begin(), valid_depths.end());
  const double ratio = depth_sample_percentile_ / 100.0;
  const size_t index = std::min(
    valid_depths.size() - 1,
    static_cast<size_t>(std::lround(ratio * static_cast<double>(valid_depths.size() - 1))));
  const double candidate_depth_m = valid_depths[index];

  if (last_accepted_depth_m_ && depth_jump_limit_m_ > 0.0) {
    const double jump_m = std::abs(candidate_depth_m - *last_accepted_depth_m_);
    if (jump_m > depth_jump_limit_m_) {
      std::ostringstream status;
      status << "depth rejected: jump=" << jump_m
             << "m > " << depth_jump_limit_m_ << "m";
      last_depth_filter_status_ = status.str();
      return std::nullopt;
    }
  }

  const double stable_tolerance_m = std::max(0.01, 0.5 * depth_jump_limit_m_);
  if (!pending_depth_m_ || std::abs(candidate_depth_m - *pending_depth_m_) > stable_tolerance_m) {
    pending_depth_m_ = candidate_depth_m;
    stable_depth_count_ = 1;
  } else {
    pending_depth_m_ = candidate_depth_m;
    stable_depth_count_ = std::min(stable_depth_frames_, stable_depth_count_ + 1);
  }

  if (stable_depth_count_ < stable_depth_frames_) {
    std::ostringstream status;
    status << "depth waiting: stable_frames=" << stable_depth_count_
           << "/" << stable_depth_frames_
           << " z=" << candidate_depth_m
           << " std=" << std_m
           << " fill=" << fill_ratio;
    last_depth_filter_status_ = status.str();
    return std::nullopt;
  }

  last_accepted_depth_m_ = candidate_depth_m;
  std::ostringstream status;
  status << "depth accepted: z=" << candidate_depth_m
         << "m std=" << std_m
         << " fill=" << fill_ratio
         << " samples=" << valid_depths.size()
         << " stable=" << stable_depth_count_ << "/" << stable_depth_frames_;
  last_depth_filter_status_ = status.str();
  return candidate_depth_m;
}

std::optional<double> CsrtIbvsNode::pixelToMeters(
  const cv::Mat & depth, const std::string & encoding, int row, int col) const
{
  (void)encoding;
  if (row < 0 || row >= depth.rows || col < 0 || col >= depth.cols) {
    return std::nullopt;
  }

  double meters = 0.0;
  switch (depth.type()) {
    case CV_16UC1: {
      const auto raw = depth.at<std::uint16_t>(row, col);
      if (raw == 0) {
        return std::nullopt;
      }
      meters = static_cast<double>(raw) * depth_unit_scale_;
      break;
    }
    case CV_32FC1: {
      const auto raw = depth.at<float>(row, col);
      if (!std::isfinite(raw) || raw <= 0.0F) {
        return std::nullopt;
      }
      meters = static_cast<double>(raw);
      break;
    }
    case CV_64FC1: {
      const auto raw = depth.at<double>(row, col);
      if (!std::isfinite(raw) || raw <= 0.0) {
        return std::nullopt;
      }
      meters = raw;
      break;
    }
    default:
      return std::nullopt;
  }

  if (!std::isfinite(meters) || meters < min_valid_depth_m_ || meters > max_valid_depth_m_) {
    return std::nullopt;
  }
  return meters;
}

void CsrtIbvsNode::publishDebugImage(
  const sensor_msgs::msg::Image::ConstSharedPtr & src_msg,
  const cv::Mat & frame,
  const cv::Rect & bbox,
  const IbvsResult & ibvs,
  bool tracking_ok) const
{
  if (!debug_image_pub_) {
    return;
  }

  cv::Mat debug = frame.clone();
  const cv::Point target_center(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);

  cv::rectangle(debug, bbox, tracking_ok ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
  cv::drawMarker(debug, target_center, cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 18, 2);
  cv::circle(debug, target_center, 4, cv::Scalar(0, 255, 255), -1);

  std::ostringstream overlay;
  overlay << stateToString()
          << " ex=" << std::fixed << std::setprecision(3) << ibvs.x_error_norm
          << " vx=" << ibvs.base_cmd.linear.x
          << " wz=" << ibvs.base_cmd.angular.z;
  cv::putText(debug, overlay.str(), cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 255), 2);

	  std::ostringstream depth_text;
	  depth_text << "depth=";
	  if (ibvs.depth_m) {
	    depth_text << std::fixed << std::setprecision(3) << *ibvs.depth_m << "m";
	  } else if (ibvs.triangulated_object_x_m) {
	    depth_text << "triangulation x="
	               << std::fixed << std::setprecision(3) << *ibvs.triangulated_object_x_m << "m";
	  } else if (!ibvs.range_status.empty()) {
	    depth_text << ibvs.range_status;
	  } else {
	    depth_text << "missing in bbox";
	  }
  if (ibvs.depth_too_close) {
    depth_text << " STOP";
  }
  cv::putText(debug, depth_text.str(), cv::Point(12, 56), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 255), 2);

  auto out_msg = cv_bridge::CvImage(src_msg->header, kBgr8, debug).toImageMsg();
  debug_image_pub_->publish(*out_msg);
}

void CsrtIbvsNode::publishDebugImageNoTrack(
  const sensor_msgs::msg::Image::ConstSharedPtr & src_msg,
  const cv::Mat & frame,
  const std::string & reason) const
{
  if (!debug_image_pub_) {
    return;
  }

  cv::Mat debug = frame.clone();
  if (debug.empty()) {
    return;
  }

  const cv::Point desired_center(
    static_cast<int>(std::lround(desired_u_ratio_ * debug.cols)),
    static_cast<int>(std::lround(desired_v_ratio_ * debug.rows)));
  cv::drawMarker(debug, desired_center, cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 18, 2);

  std::ostringstream state_text;
  state_text << stateToString() << " - " << reason;
  cv::putText(
    debug,
    state_text.str(),
    cv::Point(12, 28),
    cv::FONT_HERSHEY_SIMPLEX,
    0.65,
    cv::Scalar(255, 255, 255),
    2);

  const std::string topic_text = "init bbox: " + init_bbox_topic_;
  cv::putText(
    debug,
    topic_text,
    cv::Point(12, 56),
    cv::FONT_HERSHEY_SIMPLEX,
    0.55,
    cv::Scalar(255, 255, 255),
    2);

  auto out_msg = cv_bridge::CvImage(src_msg->header, kBgr8, debug).toImageMsg();
  debug_image_pub_->publish(*out_msg);
}

void CsrtIbvsNode::watchdog()
{
  if (state_ != TrackerState::TRACKING) {
    return;
  }

  const double age_s = (now() - last_track_stamp_).seconds();
  if (age_s > watchdog_timeout_s_) {
    resetTracker();
    state_ = TrackerState::LOST;
    min_depth_reached_ = false;
    publishCloseRangeReady(false);
    if (stop_when_lost_) {
      publishStop(true);
    }
    publishStatus("watchdog timeout; target lost", true);
  }
}

void CsrtIbvsNode::publishStatus(const std::string & text, bool force)
{
  if (!status_pub_) {
    return;
  }

  const rclcpp::Time stamp = now();
  if (!force && last_status_stamp_.nanoseconds() != 0 &&
      (stamp - last_status_stamp_).seconds() < status_period_s_) {
    return;
  }

  std_msgs::msg::String msg;
  msg.data = stateToString() + ": " + text;
  status_pub_->publish(msg);
  last_status_stamp_ = stamp;
}

void CsrtIbvsNode::publishCloseRangeReady(bool ready, bool force)
{
  if (!close_range_ready_pub_) {
    return;
  }
  if (!force && close_range_ready_sent_ == ready) {
    return;
  }

  std_msgs::msg::Bool msg;
  msg.data = ready;
  close_range_ready_pub_->publish(msg);
  close_range_ready_sent_ = ready;
}

void CsrtIbvsNode::publishStop(bool force)
{
  if (stop_sent_ && !force) {
    return;
  }

  if (enable_cmd_vel_ && cmd_vel_pub_) {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
  }
  if (enable_arm_twist_ && arm_twist_pub_) {
    geometry_msgs::msg::TwistStamped stop;
    stop.header.stamp = now();
    stop.header.frame_id = arm_command_frame_id_;
    arm_twist_pub_->publish(stop);
  }
  stop_sent_ = true;
}

std::string CsrtIbvsNode::stateToString() const
{
  switch (state_) {
    case TrackerState::WAITING_FOR_BBOX:
      return "WAITING_FOR_BBOX";
    case TrackerState::TRACKING:
      return "TRACKING";
    case TrackerState::LOST:
      return "LOST";
  }
  return "UNKNOWN";
}

}  // namespace hybrid_csrt_ibvs
