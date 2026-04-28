#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

namespace hybrid_single_target_tracker {

class LowPassFilter {
public:
  explicit LowPassFilter(double a = 0.3) : alpha_(a) {}
  void reset(double v) { value_ = v; initialized_ = true; }
  double update(double v) {
    if (!initialized_) reset(v); else value_ = alpha_ * v + (1.0 - alpha_) * value_;
    return value_;
  }
private:
  double alpha_{0.3}, value_{0.0};
  bool initialized_{false};
};

enum class DepthMode { INIT, SENSOR, VISUAL, HOLD };
static const char * modeName(DepthMode m) {
  switch (m) {
    case DepthMode::INIT: return "INIT";
    case DepthMode::SENSOR: return "SENS";
    case DepthMode::VISUAL: return "VISUAL";
    default: return "HOLD";
  }
}

struct DetectionCandidate { cv::Rect bbox; double z_m{0.0}, area{0.0}; };
struct TargetState {
  bool active{false};
  int learned_count{1};
  cv::Rect tracker_bbox, bbox;
  cv::Ptr<cv::TrackerCSRT> tracker;
  LowPassFilter fx{0.4}, fy{0.4}, fz{0.6}, fw{0.3}, fh{0.3};
  double z_m{0.0}, last_valid_z_m{0.0};
  double learned_real_w_m{0.08}, learned_real_h_m{0.08};
  double visual_fx{550.0}, visual_fy{550.0};
  double debug_sensor_z_m{0.0}, debug_visual_z_m{0.0};
  DepthMode mode{DepthMode::INIT};
};

static cv::Rect clampRect(const cv::Rect & r, const cv::Size & s)
{
  const int x0 = std::clamp(r.x, 0, s.width), y0 = std::clamp(r.y, 0, s.height);
  const int x1 = std::clamp(r.x + r.width, 0, s.width), y1 = std::clamp(r.y + r.height, 0, s.height);
  return {x0, y0, std::max(0, x1 - x0), std::max(0, y1 - y0)};
}

static cv::Rect scaleRect(const cv::Rect & r, double s)
{
  return {static_cast<int>(std::round(r.x * s)), static_cast<int>(std::round(r.y * s)),
    std::max(1, static_cast<int>(std::round(r.width * s))),
    std::max(1, static_cast<int>(std::round(r.height * s)))};
}

template<typename MsgPtr>
static cv::Mat wrapImage(const MsgPtr & msg, int type)
{
  return cv::Mat(static_cast<int>(msg->height), static_cast<int>(msg->width), type,
    const_cast<unsigned char *>(msg->data.data()), msg->step);
}

static sensor_msgs::msg::Image makeBgrImage(const std_msgs::msg::Header & header, const cv::Mat & image)
{
  const cv::Mat c = image.isContinuous() ? image : image.clone();
  sensor_msgs::msg::Image msg;
  msg.header = header;
  msg.height = static_cast<std::uint32_t>(c.rows);
  msg.width = static_cast<std::uint32_t>(c.cols);
  msg.encoding = sensor_msgs::image_encodings::BGR8;
  msg.is_bigendian = false;
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(c.step);
  msg.data.assign(c.datastart, c.dataend);
  return msg;
}

static std::string fmt(const std::string & label, double value, const std::string & unit)
{
  std::ostringstream oss;
  oss.setf(std::ios::fixed); oss.precision(3);
  oss << label << ": " << value << unit;
  return oss.str();
}

class HybridSingleTargetTracker : public rclcpp::Node {
  using ImageMsg = sensor_msgs::msg::Image;
  using CameraInfoMsg = sensor_msgs::msg::CameraInfo;

  struct Params {
    std::string color_topic{"/camera/color/image_raw"}, depth_topic{"/camera/depth/image_raw"},
      camera_info_topic{"/camera/depth/camera_info"};
    std::string position_topic{"/object/fruit_1/position"}, size_topic{"/object/fruit_1/size"},
      depth_debug_topic{"/object/fruit_1/depth_comp"}, valid_topic{"/object/fruit_1/valid"},
      raw_view_topic{"/object/picking_view"}, debug_view_topic{"/object/close_range_view"},
      output_frame_id_override{"object_1"};
    int tracker_skip{3}, detection_interval{20};
    double tracking_scale{0.5}, min_depth_trust_m{0.50}, min_detect_depth_m{0.2},
      max_detect_depth_m{1.2}, max_valid_depth_m{3.0}, min_contour_area_px{500.0},
      center_roi_ratio{0.2}, learned_size_min_m{0.03}, learned_size_max_m{0.30};
    double default_fx{550.0}, default_fy{550.0}, default_cx{320.0}, default_cy{240.0};
  } p_;

public:
  HybridSingleTargetTracker()
  : Node("hybrid_single_target_tracker", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    loadParameters();
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.best_effort();
    qos.durability_volatile();

    color_sub_ = create_subscription<ImageMsg>(p_.color_topic, qos,
      std::bind(&HybridSingleTargetTracker::colorCallback, this, std::placeholders::_1));
    depth_sub_ = create_subscription<ImageMsg>(p_.depth_topic, qos,
      std::bind(&HybridSingleTargetTracker::depthCallback, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<CameraInfoMsg>(p_.camera_info_topic, qos,
      std::bind(&HybridSingleTargetTracker::cameraInfoCallback, this, std::placeholders::_1));

    position_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(p_.position_topic, 10);
    size_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(p_.size_topic, 10);
    depth_debug_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(p_.depth_debug_topic, 10);
    valid_pub_ = create_publisher<std_msgs::msg::Bool>(p_.valid_topic, 10);
    raw_view_pub_ = create_publisher<ImageMsg>(p_.raw_view_topic, qos);
    debug_view_pub_ = create_publisher<ImageMsg>(p_.debug_view_topic, qos);

    erode_kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    fx_ = p_.default_fx; fy_ = p_.default_fy; cx_ = p_.default_cx; cy_ = p_.default_cy;
    has_intrinsics_ = fx_ > 0.0 && fy_ > 0.0;

    RCLCPP_WARN(get_logger(), "camera_info가 없어도 default intrinsics로 시작합니다.");
    RCLCPP_INFO(get_logger(), "lean hybrid tracker started: color=%s depth=%s info=%s",
      p_.color_topic.c_str(), p_.depth_topic.c_str(), p_.camera_info_topic.c_str());
  }

private:
  template<typename T> T param(const char * name, const T & fallback)
  {
    T value = fallback; get_parameter_or(name, value, fallback); return value;
  }

  void loadParameters()
  {
    p_.color_topic = param("color_topic", p_.color_topic);
    p_.depth_topic = param("depth_topic", p_.depth_topic);
    p_.camera_info_topic = param("camera_info_topic", p_.camera_info_topic);
    p_.position_topic = param("position_topic", p_.position_topic);
    p_.size_topic = param("size_topic", p_.size_topic);
    p_.depth_debug_topic = param("depth_debug_topic", p_.depth_debug_topic);
    p_.valid_topic = param("valid_topic", p_.valid_topic);
    p_.raw_view_topic = param("raw_view_topic", p_.raw_view_topic);
    p_.debug_view_topic = param("debug_view_topic", p_.debug_view_topic);
    p_.output_frame_id_override = param("output_frame_id_override", p_.output_frame_id_override);
    p_.tracker_skip = std::max(1, param("tracker_skip", p_.tracker_skip));
    p_.detection_interval = std::max(1, param("detection_interval", p_.detection_interval));
    p_.tracking_scale = std::clamp(param("tracking_scale", p_.tracking_scale), 0.1, 1.0);
    p_.min_depth_trust_m = param("min_depth_trust_m", p_.min_depth_trust_m);
    p_.min_detect_depth_m = param("min_detect_depth_m", p_.min_detect_depth_m);
    p_.max_detect_depth_m = param("max_detect_depth_m", p_.max_detect_depth_m);
    p_.max_valid_depth_m = param("max_valid_depth_m", p_.max_valid_depth_m);
    p_.min_contour_area_px = param("min_contour_area_px", p_.min_contour_area_px);
    p_.center_roi_ratio = std::clamp(param("center_roi_ratio", p_.center_roi_ratio), 0.05, 1.0);
    p_.learned_size_min_m = param("learned_size_min_m", p_.learned_size_min_m);
    p_.learned_size_max_m = param("learned_size_max_m", p_.learned_size_max_m);
    p_.default_fx = param("default_fx", p_.default_fx);
    p_.default_fy = param("default_fy", p_.default_fy);
    p_.default_cx = param("default_cx", p_.default_cx);
    p_.default_cy = param("default_cy", p_.default_cy);
  }

  void cameraInfoCallback(const CameraInfoMsg::SharedPtr msg)
  {
    fx_ = msg->k[0]; fy_ = msg->k[4]; cx_ = msg->k[2]; cy_ = msg->k[5];
    camera_frame_id_ = msg->header.frame_id; has_intrinsics_ = fx_ > 0.0 && fy_ > 0.0;
  }

  void colorCallback(const ImageMsg::ConstSharedPtr & msg)
  {
    if (!convertColorImage(msg, latest_color_bgr_)) return;
    latest_color_bgr_ = latest_color_bgr_.clone();
    latest_color_header_ = msg->header;
    has_color_ = true;
    raw_view_pub_->publish(*msg);
  }

  void depthCallback(const ImageMsg::ConstSharedPtr & msg)
  {
    if (!has_color_) return;
    ++frame_count_;

    cv::Mat depth_m;
    if (!convertDepthImage(msg, depth_m)) return;
    cv::Mat color_bgr = latest_color_bgr_.clone();
    if (color_bgr.empty()) return;

    cv::Mat depth_work = depth_m;
    if (color_bgr.size() != depth_m.size()) {
      cv::resize(depth_m, depth_resized_, color_bgr.size(), 0.0, 0.0, cv::INTER_NEAREST);
      depth_work = depth_resized_;
    }

    if (!target_.active) {
      if (!shouldDetect()) {
        publishValidity(false);
        publishDebugImage(color_bgr, latest_color_header_, nullptr);
        return;
      }
      const auto c = detectBestCandidate(depth_work);
      if (!c) {
        publishValidity(false);
        publishDebugImage(color_bgr, latest_color_header_, nullptr);
        return;
      }
      initializeTarget(color_bgr, *c);
      if (!target_.active) {
        publishValidity(false);
        publishDebugImage(color_bgr, latest_color_header_, nullptr);
        return;
      }
    } else {
      updateTracker(color_bgr);
    }

    updateHybridDepth(depth_work);
    publishTarget(msg->header);
    publishValidity(true);
    publishDebugImage(color_bgr, latest_color_header_, &target_);
  }

  bool shouldDetect() const { return (frame_count_ % p_.detection_interval) == 0; }

  bool convertColorImage(const ImageMsg::ConstSharedPtr & msg, cv::Mat & out)
  {
    const auto & enc = msg->encoding;
    auto cvt = [&](int type, int code) { cv::cvtColor(wrapImage(msg, type), out, code); return true; };
    if (enc == sensor_msgs::image_encodings::BGR8) { out = wrapImage(msg, CV_8UC3); return true; }
    if (enc == sensor_msgs::image_encodings::RGB8) return cvt(CV_8UC3, cv::COLOR_RGB2BGR);
    if (enc == sensor_msgs::image_encodings::BGRA8) return cvt(CV_8UC4, cv::COLOR_BGRA2BGR);
    if (enc == sensor_msgs::image_encodings::RGBA8) return cvt(CV_8UC4, cv::COLOR_RGBA2BGR);
    if (enc == sensor_msgs::image_encodings::MONO8) return cvt(CV_8UC1, cv::COLOR_GRAY2BGR);
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Unsupported color encoding: %s", enc.c_str());
    return false;
  }

  bool convertDepthImage(const ImageMsg::ConstSharedPtr & msg, cv::Mat & out)
  {
    const auto & enc = msg->encoding;
    if (enc == sensor_msgs::image_encodings::TYPE_16UC1 || enc == sensor_msgs::image_encodings::MONO16) {
      wrapImage(msg, CV_16UC1).convertTo(depth_meter_buffer_, CV_32FC1, 0.001);
      out = depth_meter_buffer_; return true;
    }
    if (enc == sensor_msgs::image_encodings::TYPE_32FC1) { out = wrapImage(msg, CV_32FC1); return true; }
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Unsupported depth encoding: %s", enc.c_str());
    return false;
  }

  std::optional<DetectionCandidate> detectBestCandidate(const cv::Mat & depth_m)
  {
    constexpr double kTieTol = 0.05;
    cv::inRange(depth_m, cv::Scalar(p_.min_detect_depth_m), cv::Scalar(p_.max_detect_depth_m), detection_mask_);
    cv::erode(detection_mask_, detection_mask_, erode_kernel_, cv::Point(-1, -1), 1);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(detection_mask_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::optional<DetectionCandidate> best;
    for (const auto & contour : contours) {
      const double area = cv::contourArea(contour);
      if (area < p_.min_contour_area_px) continue;
      const cv::Rect bbox = clampRect(cv::boundingRect(contour), depth_m.size());
      const double z_m = computeMedianDepth(depth_m, bbox, 1.0);
      if (z_m <= 0.0) continue;
      DetectionCandidate cand{bbox, z_m, area};
      if (!best || cand.z_m < best->z_m - kTieTol || (std::abs(cand.z_m - best->z_m) <= kTieTol && cand.area > best->area)) best = cand;
    }
    return best;
  }

  void initializeTarget(const cv::Mat & color_bgr, const DetectionCandidate & c)
  {
    target_ = TargetState{};
    target_.active = true;
    target_.bbox = clampRect(c.bbox, color_bgr.size());
    target_.tracker_bbox = target_.bbox;
    if (target_.bbox.width <= 0 || target_.bbox.height <= 0) {
      target_ = TargetState{};
      return;
    }

    const double cx = target_.bbox.x + target_.bbox.width * 0.5;
    const double cy = target_.bbox.y + target_.bbox.height * 0.5;
    target_.fx.reset(cx); target_.fy.reset(cy);
    target_.fw.reset(static_cast<double>(target_.bbox.width));
    target_.fh.reset(static_cast<double>(target_.bbox.height));
    target_.fz.reset(c.z_m);
    target_.z_m = target_.last_valid_z_m = c.z_m;
    target_.debug_sensor_z_m = target_.debug_visual_z_m = c.z_m;
    target_.mode = DepthMode::INIT;
    target_.visual_fx = fx_; target_.visual_fy = fy_;

    if (c.z_m > 0.1 && target_.visual_fx > 0.0 && target_.visual_fy > 0.0) {
      target_.learned_real_w_m = target_.bbox.width * c.z_m / target_.visual_fx;
      target_.learned_real_h_m = target_.bbox.height * c.z_m / target_.visual_fy;
      if (!std::isfinite(target_.learned_real_w_m) || target_.learned_real_w_m <= 0.0) target_.learned_real_w_m = 0.08;
      if (!std::isfinite(target_.learned_real_h_m) || target_.learned_real_h_m <= 0.0) target_.learned_real_h_m = 0.08;
    }

    target_.tracker = cv::TrackerCSRT::create();
    cv::resize(color_bgr, tracker_work_buffer_, cv::Size(), p_.tracking_scale, p_.tracking_scale, cv::INTER_LINEAR);
    target_.tracker->init(tracker_work_buffer_, scaleRect(target_.tracker_bbox, p_.tracking_scale));

    RCLCPP_INFO(get_logger(), "Target initialized: bbox=(%d,%d,%d,%d), z=%.3f m, learned_w=%.3f m",
      target_.bbox.x, target_.bbox.y, target_.bbox.width, target_.bbox.height, target_.z_m, target_.learned_real_w_m);
  }

  bool updateTracker(const cv::Mat & color_bgr)
  {
    if (!target_.active || !target_.tracker) return false;
    if ((frame_count_ % p_.tracker_skip) == 0) {
      cv::resize(color_bgr, tracker_work_buffer_, cv::Size(), p_.tracking_scale, p_.tracking_scale, cv::INTER_LINEAR);
      cv::Rect small_bbox = scaleRect(target_.tracker_bbox, p_.tracking_scale);
      const bool ok = target_.tracker->update(tracker_work_buffer_, small_bbox);
      if (ok) target_.tracker_bbox = scaleRect(small_bbox, 1.0 / p_.tracking_scale);
    }

    const double tcx = target_.tracker_bbox.x + target_.tracker_bbox.width * 0.5;
    const double tcy = target_.tracker_bbox.y + target_.tracker_bbox.height * 0.5;
    const double cx = target_.fx.update(tcx), cy = target_.fy.update(tcy);
    const double w = target_.fw.update(static_cast<double>(target_.tracker_bbox.width));
    const double h = target_.fh.update(static_cast<double>(target_.tracker_bbox.height));

    target_.bbox = clampRect({static_cast<int>(std::round(cx - w * 0.5)), static_cast<int>(std::round(cy - h * 0.5)),
      std::max(1, static_cast<int>(std::round(w))), std::max(1, static_cast<int>(std::round(h)))}, color_bgr.size());
    return target_.bbox.width > 0 && target_.bbox.height > 0;
  }

  void updateHybridDepth(const cv::Mat & depth_m)
  {
    const double sensor_z = computeMedianDepth(depth_m, target_.bbox, p_.center_roi_ratio);
    const double visual_z = estimateVisualDepth();
    double final_z = target_.last_valid_z_m;

    if (sensor_z > p_.min_depth_trust_m) {
      final_z = sensor_z; target_.mode = DepthMode::SENSOR; updateLearnedSize(sensor_z);
    } else if (visual_z > 0.0) {
      final_z = visual_z; target_.mode = DepthMode::VISUAL;
    } else {
      target_.mode = DepthMode::HOLD;
    }

    target_.z_m = target_.fz.update(final_z);
    if (target_.z_m > 0.0) target_.last_valid_z_m = target_.z_m;
    target_.debug_sensor_z_m = sensor_z;
    target_.debug_visual_z_m = visual_z;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "mode=%s sensor_z=%.3f visual_z=%.3f final_z=%.3f bbox_w=%d learned_w=%.4f",
      modeName(target_.mode), sensor_z, visual_z, target_.z_m, target_.bbox.width, target_.learned_real_w_m);
  }

  double estimateVisualDepth() const
  {
    if (target_.bbox.width <= 0 || target_.learned_real_w_m <= 0.0 || target_.visual_fx <= 0.0) return 0.0;
    return (target_.visual_fx * target_.learned_real_w_m) / static_cast<double>(target_.bbox.width);
  }

  void updateLearnedSize(double z_m)
  {
    if (target_.visual_fx <= 0.0 || target_.visual_fy <= 0.0) return;
    const double cur_w = target_.bbox.width * z_m / target_.visual_fx;
    const double cur_h = target_.bbox.height * z_m / target_.visual_fy;
    if (cur_w < p_.learned_size_min_m || cur_w > p_.learned_size_max_m) return;
    const double n = static_cast<double>(target_.learned_count);
    target_.learned_real_w_m = (target_.learned_real_w_m * n + cur_w) / (n + 1.0);
    target_.learned_real_h_m = (target_.learned_real_h_m * n + cur_h) / (n + 1.0);
    target_.learned_count = std::min(target_.learned_count + 1, 100);
  }

  double computeMedianDepth(const cv::Mat & depth_m, const cv::Rect & bbox, double roi_ratio)
  {
    const cv::Rect clipped = clampRect(bbox, depth_m.size());
    if (clipped.width <= 0 || clipped.height <= 0) return 0.0;

    const int sw = std::max(3, static_cast<int>(std::round(clipped.width * roi_ratio)));
    const int sh = std::max(3, static_cast<int>(std::round(clipped.height * roi_ratio)));
    const int cu = clipped.x + clipped.width / 2, cv = clipped.y + clipped.height / 2;
    const int rx = std::max(0, cu - sw / 2), ry = std::max(0, cv - sh / 2);
    const cv::Rect roi(rx, ry, std::min(sw, depth_m.cols - rx), std::min(sh, depth_m.rows - ry));
    if (roi.width <= 0 || roi.height <= 0) return 0.0;

    depth_samples_.clear();
    depth_samples_.reserve(static_cast<std::size_t>(roi.area()));
    for (int row = roi.y; row < roi.y + roi.height; ++row) {
      const float * row_ptr = depth_m.ptr<float>(row);
      for (int col = roi.x; col < roi.x + roi.width; ++col) {
        const float z = row_ptr[col];
        if (std::isfinite(z) && z > 0.1F && z < static_cast<float>(p_.max_valid_depth_m)) depth_samples_.push_back(z);
      }
    }
    if (depth_samples_.empty()) return 0.0;
    auto mid = depth_samples_.begin() + static_cast<std::ptrdiff_t>(depth_samples_.size() / 2U);
    std::nth_element(depth_samples_.begin(), mid, depth_samples_.end());
    return static_cast<double>(*mid);
  }

  void publishTarget(const std_msgs::msg::Header & header)
  {
    if (!target_.active) return;
    std_msgs::msg::Header out = header;
    if (!p_.output_frame_id_override.empty()) out.frame_id = p_.output_frame_id_override;
    else if (!camera_frame_id_.empty()) out.frame_id = camera_frame_id_;

    const double cx = target_.bbox.x + target_.bbox.width * 0.5;
    const double cy = target_.bbox.y + target_.bbox.height * 0.5;
    const double real_x = (cx - this->cx_) * target_.z_m / fx_;
    const double real_y = (cy - this->cy_) * target_.z_m / fy_;

    geometry_msgs::msg::PointStamped msg; msg.header = out;
    auto publish = [&](const auto & pub, double x, double y, double z) {
      msg.point.x = x; msg.point.y = y; msg.point.z = z; pub->publish(msg);
    };
    publish(position_pub_, real_x, real_y, target_.z_m);
    publish(size_pub_, target_.learned_real_w_m, target_.learned_real_h_m, static_cast<double>(target_.bbox.width));
    publish(depth_debug_pub_, target_.z_m, target_.debug_sensor_z_m, target_.debug_visual_z_m);
  }

  void publishValidity(bool ok) { std_msgs::msg::Bool msg; msg.data = ok; valid_pub_->publish(msg); }

  void publishDebugImage(const cv::Mat & color_bgr, const std_msgs::msg::Header & header, const TargetState * s)
  {
    if (!debug_view_pub_ || debug_view_pub_->get_subscription_count() == 0U) return;
    debug_view_buffer_ = color_bgr.clone();
    if (!s) {
      cv::putText(debug_view_buffer_, "NO TARGET", cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 0, 255), 2);
      debug_view_pub_->publish(makeBgrImage(header, debug_view_buffer_));
      return;
    }

    const cv::Scalar box_color = s->mode == DepthMode::SENSOR ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::rectangle(debug_view_buffer_, s->bbox, box_color, 2);
    const std::vector<std::string> lines{
      "ID: 1 [" + std::string(modeName(s->mode)) + "]",
      fmt("Final Z", s->z_m, "m"), fmt("Sens Z", s->debug_sensor_z_m, "m"),
      fmt("Vis  Z", s->debug_visual_z_m, "m"), fmt("Real W", s->learned_real_w_m * 100.0, "cm")};

    const int tx = std::min(s->bbox.x + s->bbox.width + 10, std::max(5, debug_view_buffer_.cols - 190));
    const int ty = std::max(20, s->bbox.y + 20);
    for (std::size_t i = 0; i < lines.size(); ++i) {
      const cv::Point org(tx, ty + static_cast<int>(i) * 20);
      cv::putText(debug_view_buffer_, lines[i], org, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 3);
      cv::putText(debug_view_buffer_, lines[i], org, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }
    debug_view_pub_->publish(makeBgrImage(header, debug_view_buffer_));
  }

  rclcpp::Subscription<ImageMsg>::SharedPtr color_sub_, depth_sub_;
  rclcpp::Subscription<CameraInfoMsg>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_, size_pub_, depth_debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr raw_view_pub_, debug_view_pub_;

  TargetState target_;
  int frame_count_{0};
  bool has_color_{false}, has_intrinsics_{false};
  double fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0};
  std::string camera_frame_id_;
  std_msgs::msg::Header latest_color_header_;
  cv::Mat latest_color_bgr_, erode_kernel_, tracker_work_buffer_, debug_view_buffer_, detection_mask_, depth_meter_buffer_, depth_resized_;
  std::vector<float> depth_samples_;
};

}  // namespace hybrid_single_target_tracker

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hybrid_single_target_tracker::HybridSingleTargetTracker>());
  rclcpp::shutdown();
  return 0;
}