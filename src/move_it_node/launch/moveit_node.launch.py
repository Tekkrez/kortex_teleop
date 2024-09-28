from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

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
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    
    # # Start the actual move_group node/action server
    # run_move_group_node = Node(
    #     package="moveit_ros_move_group",
    #     executable="move_group",
    #     output="screen",
    #     parameters=[moveit_config.to_dict()],
    # )

    # MoveGroupInterface demo executable
    moveit_traj_node = Node(
        name="moveit_node",
        package="move_it_node",
        executable="moveit_node",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    
    return LaunchDescription([
        # run_move_group_node,
        moveit_traj_node
        ]
    )
