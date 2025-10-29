/**
 * @file opencv_processor_node.cpp
 * @brief Color-based (HSV) object detection using OpenCV in ROS2
 *
 * Subscribes: /camera/color/image_raw (sensor_msgs/msg/Image)
 * Publishes:  /perception/detected_objects (sort_interfaces/msg/DetectedObjects)
 *             /perception/debug_image     (sensor_msgs/msg/Image)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <algorithm>

#include "sort_interfaces/msg/detected_objects.hpp"
#include "sort_interfaces/msg/bounding_box.hpp"

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <iomanip>

class HSVDetectorNode : public rclcpp::Node {
public:
  HSVDetectorNode() : Node("opencv_processor_node"), next_id_(0) {
    // Parameters (tweak in YAML)
    this->declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
    this->declare_parameter<int>("h_low", 35);
    this->declare_parameter<int>("s_low", 70);
    this->declare_parameter<int>("v_low", 70);
    this->declare_parameter<int>("h_high", 85);
    this->declare_parameter<int>("s_high", 255);
    this->declare_parameter<int>("v_high", 255);
    this->declare_parameter<int>("blur_ksize", 5);          // must be odd
    this->declare_parameter<int>("morph_ksize", 5);         // odd
    this->declare_parameter<int>("erode_iters", 1);
    this->declare_parameter<int>("dilate_iters", 2);
    this->declare_parameter<double>("min_area", 800.0);     // px
    this->declare_parameter<bool>("debug_visualization", true);

    image_topic_  = this->get_parameter("image_topic").as_string();
    h_low_        = this->get_parameter("h_low").as_int();
    s_low_        = this->get_parameter("s_low").as_int();
    v_low_        = this->get_parameter("v_low").as_int();
    h_high_       = this->get_parameter("h_high").as_int();
    s_high_       = this->get_parameter("s_high").as_int();
    v_high_       = this->get_parameter("v_high").as_int();
    blur_k_       = this->get_parameter("blur_ksize").as_int();
    morph_k_      = this->get_parameter("morph_ksize").as_int();
    erode_iters_  = this->get_parameter("erode_iters").as_int();
    dilate_iters_ = this->get_parameter("dilate_iters").as_int();
    min_area_     = this->get_parameter("min_area").as_double();
    debug_viz_    = this->get_parameter("debug_visualization").as_bool();

    // Sub/Pub
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, rclcpp::SensorDataQoS(),
        std::bind(&HSVDetectorNode::imageCb, this, std::placeholders::_1));

    det_pub_ = this->create_publisher<sort_interfaces::msg::DetectedObjects>(
        "/perception/detected_objects", 10);

    dbg_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/perception/debug_image", 10);

    RCLCPP_INFO(get_logger(), "HSV detector subscribed to: %s", image_topic_.c_str());
  }

private:
  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    cv::Mat bgr = cv_ptr->image;
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    // Optional blur
    int bk = (blur_k_ % 2 == 0) ? blur_k_ + 1 : blur_k_;
    if (bk > 1) cv::GaussianBlur(hsv, hsv, cv::Size(bk, bk), 0);

    // Threshold
    cv::Scalar low(h_low_, s_low_, v_low_);
    cv::Scalar high(h_high_, s_high_, v_high_);
    cv::Mat mask;
    cv::inRange(hsv, low, high, mask);

    // Morphology
    int mk = (morph_k_ % 2 == 0) ? morph_k_ + 1 : morph_k_;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(mk, mk));
    if (erode_iters_ > 0)  cv::erode(mask, mask, kernel, cv::Point(-1,-1), erode_iters_);
    if (dilate_iters_ > 0) cv::dilate(mask, mask, kernel, cv::Point(-1,-1), dilate_iters_);

    // Contours → boxes
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    sort_interfaces::msg::DetectedObjects out;
    out.header = msg->header;

    for (const auto& c : contours) {
      double area = cv::contourArea(c);
      if (area < min_area_) continue;

      cv::Rect box = cv::boundingRect(c);

      sort_interfaces::msg::BoundingBox bb;
      bb.id = next_id_++;
      bb.x_min = static_cast<float>(box.x);
      bb.y_min = static_cast<float>(box.y);
      bb.x_max = static_cast<float>(box.x + box.width);
      bb.y_max = static_cast<float>(box.y + box.height);
      bb.confidence = 1.0f;               // color match; you can scale by area if you want
      bb.class_name = "color_object";     // or "green_object" etc.

      out.objects.push_back(std::move(bb));
    }

    det_pub_->publish(out);

    // Debug overlay
    if (debug_viz_) {
      cv::Mat vis = bgr.clone();
      for (const auto& d : out.objects) {
        cv::rectangle(vis,
                      cv::Point(static_cast<int>(d.x_min), static_cast<int>(d.y_min)),
                      cv::Point(static_cast<int>(d.x_max), static_cast<int>(d.y_max)),
                      cv::Scalar(0,255,0), 2);
        std::ostringstream os; os << d.class_name;
        cv::putText(vis, os.str(),
                    cv::Point(static_cast<int>(d.x_min), std::max(0, static_cast<int>(d.y_min) - 5)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 2);
      }

      // Optional: show mask inset (top-left)
      cv::Mat mask_bgr; cv::cvtColor(mask, mask_bgr, cv::COLOR_GRAY2BGR);
      int w = std::min(200, vis.cols/4), h = std::min(150, vis.rows/4);
      cv::Mat roi = vis(cv::Rect(5,5,w,h));
      cv::resize(mask_bgr, roi, roi.size());
      auto msg_out = cv_bridge::CvImage(msg->header, "bgr8", vis).toImageMsg();
      dbg_pub_->publish(*msg_out);
    }
  }

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sort_interfaces::msg::DetectedObjects>::SharedPtr det_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr dbg_pub_;

  // Params
  std::string image_topic_;
  int h_low_, s_low_, v_low_, h_high_, s_high_, v_high_;
  int blur_k_, morph_k_, erode_iters_, dilate_iters_;
  double min_area_;
  bool debug_viz_;
  uint32_t next_id_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HSVDetectorNode>());
  rclcpp::shutdown();
  return 0;
}

