from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


'''
"header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
name: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7]
position: [4.75545,2.37624,1.93844,1.87679,0.81268,1.09919,3.14259]
velocity: [0,0,0,0,0,0,0]
effort: []
"
'''

def generate_launch_description():

    launch_arguments = {
        "robot_ip": "yyy.yyy.yyy.yyy",
        "use_fake_hardware": "true",
        "gripper": "robotiq_2f_85",
        "gripper_joint_name": "robotiq_85_left_knuckle_joint",
        "dof": "7",
        "gripper_max_velocity": "100.0",
        "gripper_max_force": "100.0",
        "use_internal_bus_gripper_comm": "true",
    }

    # rviz with moveit configuration
    kinematics_file = (
        get_package_share_directory("move_it_node")
        + "/config/kinematics.yaml"
    )
    moveit_config = (
        MoveItConfigsBuilder("gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config")
        .robot_description(mappings=launch_arguments)
        .robot_description_kinematics(kinematics_file)
        .to_moveit_configs()
    )

    robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="both",
    parameters=[moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics],
    )    

    rviz_config_file = (
        get_package_share_directory("move_it_node")
        + "/config/view_kortex_robot.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ]
    )

    moveit_traj_node = Node(
        name="moveit_node",
        package="move_it_node",
        executable="moveit_node",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    
    return LaunchDescription([
        rviz_node,
        robot_state_publisher_node,
        moveit_traj_node
        ]
    )
