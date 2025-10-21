/**
 * @file opencv_processor_node.cpp
 * @brief Object detection and 2D localization using OpenCV
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sort_interfaces/msg/detected_objects.hpp"
#include "sort_interfaces/msg/bounding_box.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

class OpenCVProcessorNode : public rclcpp::Node
{
public:
    OpenCVProcessorNode() : Node("opencv_processor_node"), detection_id_(0)
    {
        // Declare parameters
        this->declare_parameter("min_object_area", 1000);
        this->declare_parameter("max_object_area", 50000);
        this->declare_parameter("confidence_threshold", 0.5);
        this->declare_parameter("debug_visualization", true);

        // Get parameters
        min_object_area_ = this->get_parameter("min_object_area").as_int();
        max_object_area_ = this->get_parameter("max_object_area").as_int();
        confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
        debug_viz_ = this->get_parameter("debug_visualization").as_bool();

        // Subscriber
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10,
            std::bind(&OpenCVProcessorNode::image_callback, this, std::placeholders::_1));

        // Publishers
        detected_objects_pub_ = this->create_publisher<sort_interfaces::msg::DetectedObjects>(
            "/perception/detected_objects", 10);
        debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/perception/debug_image", 10);

        RCLCPP_INFO(this->get_logger(), "OpenCV Processor Node initialized");
        RCLCPP_INFO(this->get_logger(),
                   "Object area range: %d - %d pixels", min_object_area_, max_object_area_);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS image to OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat& image = cv_ptr->image;

        // TODO: Implement actual object detection algorithm
        // Options: Deep learning (YOLO, SSD), classical CV (contour detection), etc.
        // For now, use simple contour detection as placeholder

        auto detected_objects = detect_objects(image);

        // Publish detected objects
        sort_interfaces::msg::DetectedObjects objects_msg;
        objects_msg.header = msg->header;
        objects_msg.objects = detected_objects;
        detected_objects_pub_->publish(objects_msg);

        if (detected_objects.size() > 0) {
            RCLCPP_INFO(this->get_logger(), "Detected %zu objects", detected_objects.size());
        }

        // Publish debug visualization if enabled
        if (debug_viz_) {
            cv::Mat debug_image = image.clone();
            draw_detections(debug_image, detected_objects);

            auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_image).toImageMsg();
            debug_image_pub_->publish(*debug_msg);
        }
    }

    std::vector<sort_interfaces::msg::BoundingBox> detect_objects(const cv::Mat& image)
    {
        std::vector<sort_interfaces::msg::BoundingBox> detections;

        // TODO: Replace with actual detection algorithm (YOLO, SSD, etc.)
        // Simple contour-based detection as placeholder

        cv::Mat gray, binary;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

        // Threshold to find objects
        cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);

            if (area >= min_object_area_ && area <= max_object_area_) {
                cv::Rect bbox = cv::boundingRect(contour);

                sort_interfaces::msg::BoundingBox detection;
                detection.id = detection_id_++;
                detection.x_min = static_cast<float>(bbox.x);
                detection.y_min = static_cast<float>(bbox.y);
                detection.x_max = static_cast<float>(bbox.x + bbox.width);
                detection.y_max = static_cast<float>(bbox.y + bbox.height);
                detection.confidence = confidence_threshold_;  // Dummy confidence
                detection.class_name = "object";

                detections.push_back(detection);
            }
        }

        return detections;
    }

    void draw_detections(cv::Mat& image,
                        const std::vector<sort_interfaces::msg::BoundingBox>& detections)
    {
        for (const auto& det : detections) {
            cv::Point pt1(static_cast<int>(det.x_min), static_cast<int>(det.y_min));
            cv::Point pt2(static_cast<int>(det.x_max), static_cast<int>(det.y_max));

            cv::rectangle(image, pt1, pt2, cv::Scalar(0, 255, 0), 2);

            std::string label = det.class_name + " " +
                              std::to_string(static_cast<int>(det.confidence * 100)) + "%";
            cv::putText(image, label, pt1, cv::FONT_HERSHEY_SIMPLEX,
                       0.5, cv::Scalar(0, 255, 0), 2);
        }
    }

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sort_interfaces::msg::DetectedObjects>::SharedPtr detected_objects_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

    int min_object_area_;
    int max_object_area_;
    double confidence_threshold_;
    bool debug_viz_;
    uint32_t detection_id_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OpenCVProcessorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
