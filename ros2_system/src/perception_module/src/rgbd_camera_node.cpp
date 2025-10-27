/**
 * @file rgbd_camera_node.cpp
 * @brief RGBD camera interface node (RealSense or similar)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rclcpp/serialization.hpp>
#include <filesystem>
#include <sstream>
#include <chrono>
#include <memory>

class RGBDCameraNode : public rclcpp::Node
{
public:
    RGBDCameraNode() : Node("rgbd_camera_node"), frame_count_(0), recording_enabled_(false)
    {
        // Declare parameters
        this->declare_parameter("camera_name", "camera");
        this->declare_parameter("frame_id", "camera_link");
        this->declare_parameter("publish_rate", 30.0);
        this->declare_parameter("image_width", 640);
        this->declare_parameter("image_height", 480);
        this->declare_parameter("enable_recording", false);
        this->declare_parameter("bag_path", "");

        // Get parameters
        camera_name_ = this->get_parameter("camera_name").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        image_width_ = this->get_parameter("image_width").as_int();
        image_height_ = this->get_parameter("image_height").as_int();
        recording_enabled_ = this->get_parameter("enable_recording").as_bool();
        std::string bag_path = this->get_parameter("bag_path").as_string();

        // Publishers
        color_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10);
        depth_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10);
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/camera/pointcloud", 10);
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/camera/color/camera_info", 10);

        // Initialize bag recording if enabled
        if (recording_enabled_) {
            if (bag_path.empty()) {
                // Generate default bag path with timestamp
                auto now = std::chrono::system_clock::now();
                auto timestamp = std::chrono::system_clock::to_time_t(now);
                std::stringstream ss;
                ss << "rgbd_camera_" << timestamp;
                bag_path = ss.str();
            }

            try {
                bag_writer_ = std::make_unique<rosbag2_cpp::Writer>();

                rosbag2_storage::StorageOptions storage_options;
                storage_options.uri = bag_path;
                storage_options.storage_id = "sqlite3";

                rosbag2_cpp::ConverterOptions converter_options;
                converter_options.input_serialization_format = "cdr";
                converter_options.output_serialization_format = "cdr";

                bag_writer_->open(storage_options, converter_options);

                // Create topic metadata for RGB image
                rosbag2_storage::TopicMetadata color_topic_metadata;
                color_topic_metadata.name = "/camera/color/image_raw";
                color_topic_metadata.type = "sensor_msgs/msg/Image";
                color_topic_metadata.serialization_format = "cdr";
                bag_writer_->create_topic(color_topic_metadata);

                // Create topic metadata for depth image
                rosbag2_storage::TopicMetadata depth_topic_metadata;
                depth_topic_metadata.name = "/camera/depth/image_raw";
                depth_topic_metadata.type = "sensor_msgs/msg/Image";
                depth_topic_metadata.serialization_format = "cdr";
                bag_writer_->create_topic(depth_topic_metadata);

                // Create topic metadata for camera info
                rosbag2_storage::TopicMetadata info_topic_metadata;
                info_topic_metadata.name = "/camera/color/camera_info";
                info_topic_metadata.type = "sensor_msgs/msg/CameraInfo";
                info_topic_metadata.serialization_format = "cdr";
                bag_writer_->create_topic(info_topic_metadata);

                RCLCPP_INFO(this->get_logger(), "Recording enabled - saving to: %s", bag_path.c_str());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize bag recording: %s", e.what());
                recording_enabled_ = false;
            }
        }

        // Timer for publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
            std::bind(&RGBDCameraNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "RGBD Camera Node initialized");
        RCLCPP_INFO(this->get_logger(), "Camera: %s, Frame: %s, Rate: %.1f Hz",
                   camera_name_.c_str(), frame_id_.c_str(), publish_rate);

        // TODO: Initialize actual camera hardware (RealSense SDK, etc.)
        RCLCPP_WARN(this->get_logger(),
                   "TODO: Initialize actual camera hardware interface");
    }

    ~RGBDCameraNode()
    {
        if (recording_enabled_ && bag_writer_) {
            RCLCPP_INFO(this->get_logger(), "Closing bag file...");
            bag_writer_.reset();
        }
    }

private:
    void timer_callback()
    {
        auto timestamp = this->now();

        // TODO: Replace with actual camera data acquisition
        // For now, publish dummy data

        // Publish color image
        auto color_msg = create_dummy_color_image(timestamp);
        color_image_pub_->publish(color_msg);
        if (recording_enabled_ && bag_writer_) {
            write_message_to_bag("/camera/color/image_raw", color_msg, timestamp);
        }

        // Publish depth image
        auto depth_msg = create_dummy_depth_image(timestamp);
        depth_image_pub_->publish(depth_msg);
        if (recording_enabled_ && bag_writer_) {
            write_message_to_bag("/camera/depth/image_raw", depth_msg, timestamp);
        }

        // Publish point cloud
        auto pc_msg = create_dummy_pointcloud(timestamp);
        pointcloud_pub_->publish(pc_msg);

        // Publish camera info
        auto info_msg = create_camera_info(timestamp);
        camera_info_pub_->publish(info_msg);
        if (recording_enabled_ && bag_writer_) {
            write_message_to_bag("/camera/color/camera_info", info_msg, timestamp);
        }

        frame_count_++;

        if (frame_count_ % 100 == 0) {
            RCLCPP_DEBUG(this->get_logger(), "Published frame %lu", frame_count_);
        }
    }

    template<typename MessageT>
    void write_message_to_bag(const std::string& topic_name, const MessageT& msg, rclcpp::Time timestamp)
    {
        try {
            rclcpp::SerializedMessage serialized_msg;
            rclcpp::Serialization<MessageT> serialization;
            serialization.serialize_message(&msg, &serialized_msg);

            auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            bag_message->topic_name = topic_name;
            bag_message->time_stamp = timestamp.nanoseconds();
            bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
                new rcutils_uint8_array_t,
                [](rcutils_uint8_array_t* data) {
                    if (data) {
                        auto ret = rcutils_uint8_array_fini(data);
                        (void)ret;  // Ignore return value
                        delete data;
                    }
                }
            );

            *bag_message->serialized_data = serialized_msg.release_rcl_serialized_message();

            bag_writer_->write(bag_message);
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Failed to write message to bag: %s", e.what());
        }
    }

    sensor_msgs::msg::Image create_dummy_color_image(rclcpp::Time timestamp)
    {
        sensor_msgs::msg::Image msg;
        msg.header.stamp = timestamp;
        msg.header.frame_id = frame_id_;
        msg.height = image_height_;
        msg.width = image_width_;
        msg.encoding = "rgb8";
        msg.is_bigendian = false;
        msg.step = image_width_ * 3;
        msg.data.resize(image_height_ * msg.step);

        // TODO: Fill with actual camera data
        // Dummy: fill with gradient pattern
        for (size_t i = 0; i < msg.data.size(); i += 3) {
            msg.data[i] = (i / 3) % 255;     // R
            msg.data[i + 1] = (i / 6) % 255; // G
            msg.data[i + 2] = (i / 9) % 255; // B
        }

        return msg;
    }

    sensor_msgs::msg::Image create_dummy_depth_image(rclcpp::Time timestamp)
    {
        sensor_msgs::msg::Image msg;
        msg.header.stamp = timestamp;
        msg.header.frame_id = frame_id_;
        msg.height = image_height_;
        msg.width = image_width_;
        msg.encoding = "16UC1";  // 16-bit unsigned depth in mm
        msg.is_bigendian = false;
        msg.step = image_width_ * 2;
        msg.data.resize(image_height_ * msg.step);

        // TODO: Fill with actual depth data
        // Dummy: constant depth of 1000mm
        for (size_t i = 0; i < msg.data.size(); i += 2) {
            uint16_t depth = 1000;  // 1 meter
            msg.data[i] = depth & 0xFF;
            msg.data[i + 1] = (depth >> 8) & 0xFF;
        }

        return msg;
    }

    sensor_msgs::msg::PointCloud2 create_dummy_pointcloud(rclcpp::Time timestamp)
    {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = timestamp;
        msg.header.frame_id = frame_id_;

        // TODO: Fill with actual point cloud data from camera
        // For now, create empty point cloud structure

        msg.height = 1;
        msg.width = 0;
        msg.is_bigendian = false;
        msg.is_dense = false;
        msg.point_step = 16;  // 4 floats (x, y, z, rgb)
        msg.row_step = 0;

        return msg;
    }

    sensor_msgs::msg::CameraInfo create_camera_info(rclcpp::Time timestamp)
    {
        sensor_msgs::msg::CameraInfo msg;
        msg.header.stamp = timestamp;
        msg.header.frame_id = frame_id_;
        msg.height = image_height_;
        msg.width = image_width_;

        // TODO: Load actual camera calibration parameters
        // Dummy intrinsics for a typical camera
        msg.k[0] = 615.0;  // fx
        msg.k[2] = 320.0;  // cx
        msg.k[4] = 615.0;  // fy
        msg.k[5] = 240.0;  // cy
        msg.k[8] = 1.0;

        msg.distortion_model = "plumb_bob";
        msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};

        return msg;
    }

    // Member variables
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::string camera_name_;
    std::string frame_id_;
    int image_width_;
    int image_height_;
    size_t frame_count_;

    // Bag recording
    std::unique_ptr<rosbag2_cpp::Writer> bag_writer_;
    bool recording_enabled_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RGBDCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
