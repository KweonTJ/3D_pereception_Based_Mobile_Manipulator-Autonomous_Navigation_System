#include "hybrid_csrt_ibvs/csrt_ibvs_node.hpp"

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#ifndef HYBRID_CSRT_IBVS_HAS_OPENCV_TRACKING
#include <opencv2/video/tracking.hpp>
#endif

namespace hybrid_csrt_ibvs
{
namespace
{
constexpr char kBgr8[] = "bgr8";
}

CsrtIbvsNode::CsrtIbvsNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("csrt_ibvs_node", options)
{
  readParameters();

  const auto sensor_qos = rclcpp::SensorDataQoS();
  const auto default_qos = rclcpp::QoS(rclcpp::KeepLast(10));

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

  init_bbox_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    init_bbox_topic_, default_qos,
    [this](const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) { onInitBbox(msg); });

  if (enable_cmd_vel_) {
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, default_qos);
  }

  if (enable_arm_twist_) {
    arm_twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(arm_twist_topic_, default_qos);
  }

  if (publish_debug_image_) {
    debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, default_qos);
  }

  status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, default_qos);

  last_track_stamp_ = now();
  last_status_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), [this]() { watchdog(); });

  publishStatus("ready; publish bbox [x, y, w, h] to " + init_bbox_topic_, true);
}

void CsrtIbvsNode::readParameters()
{
  image_topic_ = declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
  depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
  camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");
  init_bbox_topic_ = declare_parameter<std::string>("init_bbox_topic", "/target/init_bbox");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
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
  max_linear_x_ = declare_parameter<double>("max_linear_x", 0.12);
  max_angular_z_ = declare_parameter<double>("max_angular_z", 0.55);
  max_arm_linear_ = declare_parameter<double>("max_arm_linear", 0.025);

  depth_roi_radius_px_ = declare_parameter<int>("depth_roi_radius_px", 6);
  depth_unit_scale_ = declare_parameter<double>("depth_unit_scale", 0.001);
  min_valid_depth_m_ = declare_parameter<double>("min_valid_depth_m", 0.12);
  max_valid_depth_m_ = declare_parameter<double>("max_valid_depth_m", 3.0);
  emergency_stop_depth_m_ = declare_parameter<double>("emergency_stop_depth_m", 0.18);
  max_depth_stamp_age_s_ = declare_parameter<double>("max_depth_stamp_age_s", 0.35);

  loss_frame_limit_ = std::max(1, loss_frame_limit_);
  min_bbox_size_px_ = std::max(2, min_bbox_size_px_);
  depth_roi_radius_px_ = std::max(1, depth_roi_radius_px_);
  max_linear_x_ = std::max(0.0, max_linear_x_);
  max_angular_z_ = std::max(0.0, max_angular_z_);
  max_arm_linear_ = std::max(0.0, max_arm_linear_);
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

  {
    std::lock_guard<std::mutex> lock(bbox_mutex_);
    pending_init_bbox_ = cv::Rect(x, y, w, h);
  }

  publishStatus("bbox received; tracker will initialize on the next image", true);
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

void CsrtIbvsNode::onDepth(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
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

void CsrtIbvsNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, kBgr8);
  } catch (const cv_bridge::Exception & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1500, "image cv_bridge conversion failed: %s", ex.what());
    return;
  }

  const cv::Mat & frame = cv_ptr->image;
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

    if (lost_count_ >= loss_frame_limit_) {
      resetTracker();
      state_ = TrackerState::LOST;
      if (stop_when_lost_) {
        publishStop(true);
      }
      publishStatus("target lost; publish a fresh bbox to reinitialize", true);
    }
    return;
  }

  lost_count_ = 0;
  last_track_stamp_ = stamp;
  state_ = TrackerState::TRACKING;

  const auto ibvs = computeIbvsCommand(*clipped_box, frame.size(), stamp);

  if (enable_cmd_vel_ && cmd_vel_pub_) {
    cmd_vel_pub_->publish(ibvs.base_cmd);
    stop_sent_ = false;
  }

  if (enable_arm_twist_ && arm_twist_pub_) {
    arm_twist_pub_->publish(ibvs.arm_cmd);
  }

  if (publish_debug_image_) {
    publishDebugImage(msg, frame, *clipped_box, ibvs, true);
  }

  std::ostringstream status;
  status << "tracking bbox=[" << clipped_box->x << "," << clipped_box->y << ","
         << clipped_box->width << "," << clipped_box->height << "]"
         << " ex=" << ibvs.x_error_norm
         << " ez=";
  if (ibvs.depth_m) {
    status << (*ibvs.depth_m - desired_depth_m_);
  } else {
    status << "area";
  }
  status << " vx=" << ibvs.base_cmd.linear.x
         << " wz=" << ibvs.base_cmd.angular.z;
  publishStatus(status.str());
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

    const cv::Mat roi = hsv(bbox);
    const cv::Mat roi_mask = mask(bbox);
    const int channels[] = {0, 1};
    const int hist_size[] = {30, 32};
    const float h_range[] = {0.0F, 180.0F};
    const float s_range[] = {0.0F, 256.0F};
    const float * ranges[] = {h_range, s_range};

    cv::calcHist(&roi, 1, channels, roi_mask, tracker_hist_, 2, hist_size, ranges);
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
    publishStatus(std::string(trackerBackendName()) + " initialized", true);
    return true;
  } catch (const cv::Exception & ex) {
    resetTracker();
    state_ = TrackerState::LOST;
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
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, cv::Scalar(0, 30, 20), cv::Scalar(180, 255, 255), mask);

  const int channels[] = {0, 1};
  const float h_range[] = {0.0F, 180.0F};
  const float s_range[] = {0.0F, 256.0F};
  const float * ranges[] = {h_range, s_range};
  cv::calcBackProject(&hsv, 1, channels, tracker_hist_, back_project, ranges);
  cv::bitwise_and(back_project, mask, back_project);

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
#endif
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

CsrtIbvsNode::IbvsResult CsrtIbvsNode::computeIbvsCommand(
  const cv::Rect & bbox, const cv::Size & image_size, const rclcpp::Time & stamp)
{
  IbvsResult result;
  result.arm_cmd.header.stamp = stamp;
  result.arm_cmd.header.frame_id = arm_command_frame_id_;

  const double target_u = static_cast<double>(bbox.x) + 0.5 * static_cast<double>(bbox.width);
  const double target_v = static_cast<double>(bbox.y) + 0.5 * static_cast<double>(bbox.height);
  const double desired_u = desired_u_ratio_ * static_cast<double>(image_size.width);
  const double desired_v = desired_v_ratio_ * static_cast<double>(image_size.height);
  result.area_ratio = static_cast<double>(bbox.width * bbox.height) /
    static_cast<double>(std::max(1, image_size.width * image_size.height));

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

  double angular_z = 0.0;
  if (std::abs(result.x_error_norm) > x_deadband_norm_) {
    // Camera x increases to the right. Positive ROS base yaw turns left, so the sign is negative.
    angular_z = -yaw_gain_ * result.x_error_norm;
  }

  double linear_x = 0.0;
  result.depth_m = estimateDepthMeters(bbox, image_size, stamp);

  if (result.depth_m) {
    if (*result.depth_m < emergency_stop_depth_m_) {
      result.depth_too_close = true;
      linear_x = 0.0;
      angular_z = 0.0;
    } else {
      const double depth_error = *result.depth_m - desired_depth_m_;
      if (std::abs(depth_error) > depth_deadband_m_) {
        linear_x = linear_gain_ * depth_error;
      }
    }
  } else if (use_area_fallback_) {
    const double area_error = desired_area_ratio_ - result.area_ratio;
    if (std::abs(area_error) > area_deadband_ratio_) {
      linear_x = area_gain_ * area_error;
    }
  }

  if (linear_x > 0.0 && approach_yaw_gate_norm_ > 1e-6) {
    const double gate = clampValue(1.0 - std::abs(result.x_error_norm) / approach_yaw_gate_norm_, 0.0, 1.0);
    linear_x *= gate;
  }

  if (!allow_reverse_ && linear_x < 0.0) {
    linear_x = 0.0;
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
  const cv::Rect & bbox, const cv::Size & image_size, const rclcpp::Time & image_stamp) const
{
  if (!use_depth_) {
    return std::nullopt;
  }

  std::optional<DepthSample> sample;
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    sample = latest_depth_;
  }

  if (!sample || sample->image.empty()) {
    return std::nullopt;
  }

  if (max_depth_stamp_age_s_ > 0.0 && image_stamp.nanoseconds() != 0 && sample->stamp.nanoseconds() != 0) {
    const double age_s = std::abs((image_stamp - sample->stamp).seconds());
    if (age_s > max_depth_stamp_age_s_) {
      return std::nullopt;
    }
  }

  const double scale_x = static_cast<double>(sample->image.cols) / static_cast<double>(std::max(1, image_size.width));
  const double scale_y = static_cast<double>(sample->image.rows) / static_cast<double>(std::max(1, image_size.height));
  const int center_x = static_cast<int>(std::lround((bbox.x + 0.5 * bbox.width) * scale_x));
  const int center_y = static_cast<int>(std::lround((bbox.y + 0.5 * bbox.height) * scale_y));

  const cv::Rect depth_bounds(0, 0, sample->image.cols, sample->image.rows);
  const cv::Rect roi(
    center_x - depth_roi_radius_px_,
    center_y - depth_roi_radius_px_,
    2 * depth_roi_radius_px_ + 1,
    2 * depth_roi_radius_px_ + 1);
  const cv::Rect clipped_roi = roi & depth_bounds;
  if (clipped_roi.empty()) {
    return std::nullopt;
  }

  std::vector<double> valid_depths;
  valid_depths.reserve(static_cast<size_t>(clipped_roi.width * clipped_roi.height));

  for (int r = clipped_roi.y; r < clipped_roi.y + clipped_roi.height; ++r) {
    for (int c = clipped_roi.x; c < clipped_roi.x + clipped_roi.width; ++c) {
      const auto meters = pixelToMeters(sample->image, sample->encoding, r, c);
      if (meters) {
        valid_depths.push_back(*meters);
      }
    }
  }

  if (valid_depths.empty()) {
    return std::nullopt;
  }

  std::sort(valid_depths.begin(), valid_depths.end());
  return valid_depths[valid_depths.size() / 2];
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
  const cv::Point desired_center(
    static_cast<int>(std::lround(desired_u_ratio_ * debug.cols)),
    static_cast<int>(std::lround(desired_v_ratio_ * debug.rows)));

  cv::rectangle(debug, bbox, tracking_ok ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
  cv::circle(debug, target_center, 4, cv::Scalar(0, 255, 255), -1);
  cv::drawMarker(debug, desired_center, cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 18, 2);
  cv::line(debug, desired_center, target_center, cv::Scalar(255, 255, 0), 1);

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
  } else {
    depth_text << "area fallback";
  }
  if (ibvs.depth_too_close) {
    depth_text << " STOP";
  }
  cv::putText(debug, depth_text.str(), cv::Point(12, 56), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 255), 2);

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
