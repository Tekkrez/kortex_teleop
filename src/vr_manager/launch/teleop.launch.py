from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Launch tcp server to communicate with Unity node
    tcp_server = Node(
        name="tcp_server",
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        output="screen",
        emulate_tty=True,
        parameters=[{"ROS_IP": "192.168.1.171"}, {"ROS_TCP_PORT": 10000}]
    )
    robot_node = Node(
        name="robot_node",
        package="gen3_control",
        executable="gen3_vel_control",
        output="screen",
        emulate_tty=True,
    )
    camera_interceptor_node = Node(
        name = "camera_interceptor_node",
        package = "vr_manager",
        executable = "camera_interceptor_node",
        output = "screen",
        emulate_tty = True,
    )
    # Take controller position and generate desired position
    des_pose = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        FindPackageShare("vr_manager"),'/launch/','pose_viz.launch.py']))
    # Take webcam image and publish it
    image_pub = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        FindPackageShare("vr_manager"),'/launch/','camera_align.launch.py']))
    # Find required joint velocities from position error
    move_it_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        FindPackageShare("move_it_node"),'/launch/','moveit_vel_control_node.launch.py'])) 
    
    return LaunchDescription([
        tcp_server,
        des_pose,
        image_pub,
        move_it_node,
        camera_interceptor_node
        # robot_node
        ]
    )
