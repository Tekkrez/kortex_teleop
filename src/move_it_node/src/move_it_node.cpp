//ROS
#include <rclcpp/rclcpp.hpp>
//Basic Move it stuff
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
//For goal pose?
#include <moveit/kinematic_constraints/utils.h>
//MSGs
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
using std::placeholders::_1;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_traj_node");

Eigen::VectorXd latest_q(7);
Eigen::VectorXd latest_q_dot(7);

std::vector<double> positions;
std::vector<double> velocities;
// Flags
bool new_joint_state = false;

void jointState_Callback(const sensor_msgs::msg::JointState& msg)
{
    positions = msg.position;
    velocities = msg.velocity;

    std::copy(positions.begin(),positions.end(),latest_q.data());
    std::copy(velocities.begin(),velocities.end(),latest_q_dot.data());
    new_joint_state = true;
}

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto traj_pub_node = rclcpp::Node::make_shared("traj_pub_node", node_options);
    //Subscribers
    rclcpp::QoS sub_qos(3);
    sub_qos.best_effort();
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointState_sub;
    jointState_sub = traj_pub_node->create_subscription<sensor_msgs::msg::JointState>("joint_states",sub_qos,jointState_Callback);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(traj_pub_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    robot_model_loader::RobotModelLoader robot_model_loader(traj_pub_node, "robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    const std::string PLANNING_GROUP = "manipulator";
    moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup(PLANNING_GROUP);

    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.3;
    target_pose.position.y = 0.3;
    target_pose.position.z = 0.3;


    while(rclcpp::ok())
    {
        if(new_joint_state)
        {   
            //Reset flag
            new_joint_state=false;
            //Update current Joint State
            current_state.setJointGroupPositions(joint_model_group,positions);
            current_state.setFromIK(joint_model_group,target_pose,"end_effector_link",0.0);
            std::vector<double> desired_state;
            current_state.copyJointGroupPositions(joint_model_group,desired_state);
            // RCLCPP_INFO_STREAM(LOGGER,"Target_position: \n" << desired_state << "\n");
            // std::cout << "Target_position: \n" << desired_state << "\n";
            Eigen::Isometry3d end_effector_state = current_state.getGlobalLinkTransform("end_effector_link");
            RCLCPP_INFO_STREAM(LOGGER,"Translation: \n" << end_effector_state.translation()<<"\n");
            RCLCPP_INFO_STREAM(LOGGER,"Rotation: \n" << end_effector_state.rotation()<<"\n");

        }
    }
    rclcpp::shutdown();
    return 0;
}