from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    vr_manager_node = Node(
        name="vr_manager_node",
        package="vr_manager",
        executable="vr_manager_node",
        output="screen",
        emulate_tty=True,
    )    

    rviz_config_file = (
        get_package_share_directory("vr_manager")
        + "/config/pose_viz_rviz_config.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_pose_viz",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    tf2_static_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="rviz2_pose_viz",
        output="screen",
        arguments=["--frame-id", "world","--child-frame-id","vr_world"],
    )
    
    return LaunchDescription([
        vr_manager_node,
        rviz_node,
        tf2_static_publisher_node
        ]
    )
