from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Launch tcp server to communicate with Unity node
    grasp_generator = Node(
        name="grasp_generator",
        package="contact_graspnet_pt_ros2",
        executable="grasp_generator",
        output="screen",
        emulate_tty=True
    )
    instance_segmenter = Node(
        name="instance_segmenter",
        package="yolo_fastsam_ros2",
        executable="grasp_requester",
        output="screen",
        emulate_tty=True,
    )
    grasp_executor =  Node(
        name="grasp_executor",
        package="vr_manager",
        executable="grasp_executor_node",
        output="screen",
        emulate_tty=True,
    )
    camera_calibrator =  Node(
        name="camera_calibrator",
        package="vr_manager",
        executable="camera_alignment_node",
        output="screen",
        emulate_tty=True,
    )
    return LaunchDescription([
        grasp_generator,
        instance_segmenter,
        grasp_executor,
        camera_calibrator
        ]
    )
