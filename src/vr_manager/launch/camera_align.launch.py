from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # right_camera_config = os.path.join(
    #     get_package_share_directory("vr_manager"),
    #     'config',
    #     'camera_align_config.yaml')
    # Launch tcp server to communicate with Unity node
    right_camera = Node(
        namespace="head",
        name="right_camera",
        package="realsense2_camera",
        executable="realsense2_camera_node",
        output="screen",
        emulate_tty=True,
        # parameters=[right_camera_config,{"camera_namespace": "head"}]
        parameters=[
            {"rgb_camera.color_profile": '1280x720x30'},
            {"enable_color": True},
            {"temporal_filter.enable": True},
            {"spatial_filter.enable": True},
            {"align_depth.enable": True},
            {"enable_infra1": False},
            {"enable_infra2": False},
            {"base_frame_id": "head_camera"},
            {"publish_tf": False}]
    )
    tf2_static_publisher_node_1 = Node(
        namespace="head",
        package="tf2_ros",
        executable="static_transform_publisher",
        name="depth_camera_tf_pub",
        output="screen",
        arguments=["--x", "0","--y", "0","--z", "0",
                   "--qx", "0","--qy", "0","--qz", "0","--qw", "1",
                   "--frame-id", "head_camera","--child-frame-id","head_camera_depth_frame"],
    )    
    tf2_static_publisher_node_2 = Node(
        namespace="head",
        package="tf2_ros",
        executable="static_transform_publisher",
        name="color_camera_tf_pub",
        output="screen",
        arguments=["--x", "-0.00028015","--y", "0.014936","--z", "0.000280153",
                   "--qx", "-0.00081228","--qy", "0.000903","--qz", "0.0034416","--qw", "0.99999",
                   "--frame-id", "head_camera","--child-frame-id","head_camera_colour_frame"],
    )

    tf2_static_publisher_node_3 = Node(
        namespace="head",
        package="tf2_ros",
        executable="static_transform_publisher",
        name="depth_camera_tf_pub",
        output="screen",
        arguments=["--x", "0","--y", "0","--z", "0",
                   "--qx", "-0.5","--qy", "0.5","--qz", "-0.5","--qw", "0.5",
                   "--frame-id", "head_camera_depth_frame","--child-frame-id","head_camera_ros_depth_frame"],
    )    
    tf2_static_publisher_node_4 = Node(
        namespace="head",
        package="tf2_ros",
        executable="static_transform_publisher",
        name="color_camera_tf_pub",
        output="screen",
        arguments=["--x", "0","--y", "0","--z", "0",
                   "--qx", "-0.5","--qy", "0.5","--qz", "-0.5","--qw", "0.5",
                   "--frame-id", "head_camera_colour_frame","--child-frame-id","head_camera_ros_colour_frame"],
    )

    kinova_vision = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        FindPackageShare("kinova_vision"),'/launch/','kinova_vision.launch.py']))
    
    aruco_node_1 = Node(
        namespace="head",
        name="aruco_node_1",
        package="ros2_aruco",
        executable="aruco_node",
        parameters=[
            {"marker_size": 0.05},
            {"aruco_dictionary_id": "DICT_5X5_250"},
            {"image_topic": "/head/right_camera/color/image_raw"},
            {"camera_info_topic": "/head/right_camera/color/camera_info"},
            {"camera_frame": "head_camera_colour_frame"},
        ]
    )

    aruco_node_2 = Node(
        namespace="arm",
        name="aruco_node_2",
        package="ros2_aruco",
        executable="aruco_node",
        parameters=[
            {"marker_size": 0.05},
            {"aruco_dictionary_id": "DICT_5X5_250"},
            {"image_topic": "/camera/color/image_raw"},
            {"camera_info_topic": "/camera/color/camera_info"},
            
        ]
    )


    return LaunchDescription([
        right_camera,
        kinova_vision,
        aruco_node_1,
        aruco_node_2,
        tf2_static_publisher_node_1,
        tf2_static_publisher_node_2,
        tf2_static_publisher_node_3,
        tf2_static_publisher_node_4
        ]
    )