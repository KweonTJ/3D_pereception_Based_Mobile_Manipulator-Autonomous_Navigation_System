#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <opencv2/core.hpp>

#ifdef HYBRID_CSRT_IBVS_HAS_OPENCV_TRACKING
#include <opencv2/tracking.hpp>
#endif

namespace hybrid_csrt_ibvs
{

class CsrtIbvsNode final : public rclcpp::Node
{
public:
  explicit CsrtIbvsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum class TrackerState
  {
    WAITING_FOR_BBOX,
    TRACKING,
    LOST
  };

  struct DepthSample
  {
    cv::Mat image;
    std::string encoding;
    rclcpp::Time stamp;
  };

  struct IbvsResult
  {
    geometry_msgs::msg::Twist base_cmd;
    geometry_msgs::msg::TwistStamped arm_cmd;
    std::optional<double> depth_m;
    double x_error_norm{0.0};
    double y_error_norm{0.0};
    double area_ratio{0.0};
    bool depth_too_close{false};
  };

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void onDepth(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
  void onInitBbox(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg);
  void watchdog();

  void readParameters();
  bool initializeTracker(const cv::Mat & frame, const cv::Rect & bbox);
  bool updateTracker(const cv::Mat & frame, cv::Rect & bbox);
  bool hasTracker() const;
  void resetTracker();
  const char * trackerBackendName() const;
  std::optional<cv::Rect> sanitizeBox(const cv::Rect & bbox, const cv::Size & image_size) const;
  IbvsResult computeIbvsCommand(const cv::Rect & bbox, const cv::Size & image_size, const rclcpp::Time & stamp);
  std::optional<double> estimateDepthMeters(const cv::Rect & bbox, const cv::Size & image_size, const rclcpp::Time & image_stamp) const;
  std::optional<double> pixelToMeters(const cv::Mat & depth, const std::string & encoding, int row, int col) const;
  void publishDebugImage(const sensor_msgs::msg::Image::ConstSharedPtr & src_msg, const cv::Mat & frame, const cv::Rect & bbox, const IbvsResult & ibvs, bool tracking_ok) const;
  void publishStatus(const std::string & text, bool force = false);
  void publishStop(bool force = false);
  std::string stateToString() const;

  template<typename T>
  T clampValue(T value, T min_value, T max_value) const
  {
    return std::max(min_value, std::min(value, max_value));
  }

  // Topic parameters
  std::string image_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string init_bbox_topic_;
  std::string tracked_bbox_topic_;
  std::string cmd_vel_topic_;
  std::string arm_twist_topic_;
  std::string debug_image_topic_;
  std::string status_topic_;
  std::string arm_command_frame_id_;

  // Behavior parameters
  bool use_depth_{true};
  bool use_area_fallback_{true};
  bool enable_cmd_vel_{true};
  bool enable_arm_twist_{false};
  bool publish_debug_image_{true};
  bool stop_when_lost_{true};
  bool allow_reverse_{true};
  int loss_frame_limit_{8};
  int min_bbox_size_px_{12};
  double watchdog_timeout_s_{0.7};
  double status_period_s_{0.5};

  // IBVS parameters
  double desired_u_ratio_{0.5};
  double desired_v_ratio_{0.5};
  double desired_depth_m_{0.45};
  double desired_area_ratio_{0.06};
  double yaw_gain_{1.35};
  double linear_gain_{0.65};
  double area_gain_{1.2};
  double arm_lateral_gain_{0.05};
  double arm_vertical_gain_{0.05};
  double arm_depth_gain_{0.03};
  double x_deadband_norm_{0.035};
  double y_deadband_norm_{0.035};
  double depth_deadband_m_{0.035};
  double area_deadband_ratio_{0.01};
  double approach_yaw_gate_norm_{0.28};
  double max_linear_x_{0.12};
  double max_angular_z_{0.55};
  double max_arm_linear_{0.025};

  // Depth parameters
  int depth_roi_radius_px_{6};
  double depth_unit_scale_{0.001};
  double min_valid_depth_m_{0.12};
  double max_valid_depth_m_{3.0};
  double emergency_stop_depth_m_{0.18};
  double max_depth_stamp_age_s_{0.35};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr init_bbox_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr arm_twist_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr tracked_bbox_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

#ifdef HYBRID_CSRT_IBVS_HAS_OPENCV_TRACKING
  cv::Ptr<cv::TrackerCSRT> tracker_;
#else
  cv::Mat tracker_hist_;
  cv::Rect tracker_window_;
  bool tracker_initialized_{false};
#endif
  TrackerState state_{TrackerState::WAITING_FOR_BBOX};
  int lost_count_{0};
  rclcpp::Time last_track_stamp_;
  rclcpp::Time last_status_stamp_;
  bool stop_sent_{true};

  mutable std::mutex bbox_mutex_;
  std::optional<cv::Rect> pending_init_bbox_;

  mutable std::mutex depth_mutex_;
  std::optional<DepthSample> latest_depth_;

  mutable std::mutex camera_info_mutex_;
  bool have_camera_info_{false};
  double fx_{0.0};
  double fy_{0.0};
  double camera_cx_{0.0};
  double camera_cy_{0.0};
};

}  // namespace hybrid_csrt_ibvs
