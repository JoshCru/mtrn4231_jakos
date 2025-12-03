from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Launch arguments
    yolo_weights_arg = DeclareLaunchArgument(
        'yolo_weights',
        default_value='/runs/detect/weights/best.pt',
        description='Path to YOLO .pt model'
    )

    target_class_arg = DeclareLaunchArgument(
        'target_class_name',
        default_value='red_object',
        description='Target class name for YOLO'
    )

    # 1️⃣ RealSense camera node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        output='screen',
        parameters=[
            {"align_depth.enable": True},
            {"enable_color": True},
            {"enable_depth": True},
            {"pointcloud.enable": True},
        ]
    )

    # 2️⃣ Static TF (base_link → camera_link)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera',
        output='screen',
        arguments=[
            "1.30938", "0.0206053", "0.670571",    # XYZ
            "-0.398486", "0.00254305", "0.917119", "0.00974536",   # Quaternion XYZW
            "base_link", "camera_link"
        ]
    )

    # 3️⃣ YOLO Detection Node
    yolo_node = Node(
        package='perception_package',
        executable='object_detect_yolo',
        name='object_detect_yolo',
        output='screen',
        parameters=[
            {"yolo_weights": LaunchConfiguration("yolo_weights")},
            {"target_class_name": LaunchConfiguration("target_class_name")},
            {"conf_thres": 0.25},
            {"debug_view": True},
            {"show_mask": False},   # set True to see HSV circle mask
        ]
    )

    return LaunchDescription([
        yolo_weights_arg,
        target_class_arg,
        realsense_node,
        static_tf_node,
        yolo_node
    ])
