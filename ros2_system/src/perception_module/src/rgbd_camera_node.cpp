/**
 * @file rgbd_camera_node.cpp
 * @brief RGBD camera subscriber and recorder node for RealSense cameras
 *
 * This node subscribes to RGB and depth image topics from a RealSense camera
 * and optionally records them to a ROS bag file.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
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
    RGBDCameraNode() : Node("rgbd_camera_node"),
                       color_frame_count_(0),
                       depth_frame_count_(0),
                       recording_enabled_(false)
    {
        // Declare parameters
        this->declare_parameter("color_topic", "/camera/color/image_raw");
        this->declare_parameter("depth_topic", "/camera/depth/image_raw");
        this->declare_parameter("color_info_topic", "/camera/color/camera_info");
        this->declare_parameter("depth_info_topic", "/camera/depth/camera_info");
        this->declare_parameter("enable_recording", false);
        this->declare_parameter("bag_path", "");

        // Get parameters
        std::string color_topic = this->get_parameter("color_topic").as_string();
        std::string depth_topic = this->get_parameter("depth_topic").as_string();
        std::string color_info_topic = this->get_parameter("color_info_topic").as_string();
        std::string depth_info_topic = this->get_parameter("depth_info_topic").as_string();
        recording_enabled_ = this->get_parameter("enable_recording").as_bool();
        std::string bag_path = this->get_parameter("bag_path").as_string();

        // Store topic names for bag recording
        color_topic_ = color_topic;
        depth_topic_ = depth_topic;
        color_info_topic_ = color_info_topic;
        depth_info_topic_ = depth_info_topic;

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
                color_topic_metadata.name = color_topic;
                color_topic_metadata.type = "sensor_msgs/msg/Image";
                color_topic_metadata.serialization_format = "cdr";
                bag_writer_->create_topic(color_topic_metadata);

                // Create topic metadata for depth image
                rosbag2_storage::TopicMetadata depth_topic_metadata;
                depth_topic_metadata.name = depth_topic;
                depth_topic_metadata.type = "sensor_msgs/msg/Image";
                depth_topic_metadata.serialization_format = "cdr";
                bag_writer_->create_topic(depth_topic_metadata);

                // Create topic metadata for color camera info
                rosbag2_storage::TopicMetadata color_info_topic_metadata;
                color_info_topic_metadata.name = color_info_topic;
                color_info_topic_metadata.type = "sensor_msgs/msg/CameraInfo";
                color_info_topic_metadata.serialization_format = "cdr";
                bag_writer_->create_topic(color_info_topic_metadata);

                // Create topic metadata for depth camera info
                rosbag2_storage::TopicMetadata depth_info_topic_metadata;
                depth_info_topic_metadata.name = depth_info_topic;
                depth_info_topic_metadata.type = "sensor_msgs/msg/CameraInfo";
                depth_info_topic_metadata.serialization_format = "cdr";
                bag_writer_->create_topic(depth_info_topic_metadata);

                RCLCPP_INFO(this->get_logger(), "Recording enabled - saving to: %s", bag_path.c_str());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize bag recording: %s", e.what());
                recording_enabled_ = false;
            }
        }

        // Create subscribers for camera topics
        color_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            color_topic, 10,
            std::bind(&RGBDCameraNode::color_image_callback, this, std::placeholders::_1));

        depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic, 10,
            std::bind(&RGBDCameraNode::depth_image_callback, this, std::placeholders::_1));

        color_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            color_info_topic, 10,
            std::bind(&RGBDCameraNode::color_info_callback, this, std::placeholders::_1));

        depth_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            depth_info_topic, 10,
            std::bind(&RGBDCameraNode::depth_info_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "RGBD Camera Node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to:");
        RCLCPP_INFO(this->get_logger(), "  Color: %s", color_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Depth: %s", depth_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Color Info: %s", color_info_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Depth Info: %s", depth_info_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Recording: %s", recording_enabled_ ? "ENABLED" : "DISABLED");
    }

    ~RGBDCameraNode()
    {
        if (recording_enabled_ && bag_writer_) {
            RCLCPP_INFO(this->get_logger(), "Closing bag file...");
            RCLCPP_INFO(this->get_logger(), "Recorded %lu color frames and %lu depth frames",
                       color_frame_count_, depth_frame_count_);
            bag_writer_.reset();
        }
    }

private:
    void color_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        color_frame_count_++;

        if (color_frame_count_ % 30 == 0) {
            RCLCPP_INFO(this->get_logger(), "Received color frame %lu (%dx%d, %s)",
                       color_frame_count_, msg->width, msg->height, msg->encoding.c_str());
        }

        if (recording_enabled_ && bag_writer_) {
            write_message_to_bag(color_topic_, *msg, msg->header.stamp);
        }
    }

    void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        depth_frame_count_++;

        if (depth_frame_count_ % 30 == 0) {
            RCLCPP_INFO(this->get_logger(), "Received depth frame %lu (%dx%d, %s)",
                       depth_frame_count_, msg->width, msg->height, msg->encoding.c_str());
        }

        if (recording_enabled_ && bag_writer_) {
            write_message_to_bag(depth_topic_, *msg, msg->header.stamp);
        }
    }

    void color_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (recording_enabled_ && bag_writer_) {
            write_message_to_bag(color_info_topic_, *msg, msg->header.stamp);
        }
    }

    void depth_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (recording_enabled_ && bag_writer_) {
            write_message_to_bag(depth_info_topic_, *msg, msg->header.stamp);
        }
    }

    template<typename MessageT>
    void write_message_to_bag(const std::string& topic_name, const MessageT& msg,
                              const builtin_interfaces::msg::Time& stamp)
    {
        try {
            rclcpp::SerializedMessage serialized_msg;
            rclcpp::Serialization<MessageT> serialization;
            serialization.serialize_message(&msg, &serialized_msg);

            auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            bag_message->topic_name = topic_name;

            // Convert ROS time to nanoseconds
            bag_message->time_stamp = rclcpp::Time(stamp).nanoseconds();

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

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;

    std::string color_topic_;
    std::string depth_topic_;
    std::string color_info_topic_;
    std::string depth_info_topic_;

    size_t color_frame_count_;
    size_t depth_frame_count_;

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
