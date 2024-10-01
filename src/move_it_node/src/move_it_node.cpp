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
#include <visualization_msgs/msg/marker.hpp>
// Util
#include <robot_util.h>
#include <chrono>
using std::placeholders::_1;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_traj_node");

Eigen::VectorXd latest_q(7);
Eigen::VectorXd latest_q_dot(7);

std::vector<double> positions;
std::vector<double> velocities;

std::vector<double> desired_state;
Eigen::VectorXd desired_q(7);

// Flags
bool new_joint_state = false;
bool traj_gen_needed = false;

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
    //Rviz2 marker pub
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    marker_pub = traj_pub_node->create_publisher<visualization_msgs::msg::Marker>("visualization_marker",1);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(traj_pub_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    robot_model_loader::RobotModelLoader robot_model_loader(traj_pub_node, "robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    //Add collision object to the scene
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;
    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 2;
    primitive.dimensions[primitive.BOX_Y] = 1;
    primitive.dimensions[primitive.BOX_Z] = 0.2;
    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.8;
    box_pose.position.y = -0.3;
    box_pose.position.z = -0.1;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    visualization_msgs::msg::Marker box;
    box.header.frame_id = collision_object.header.frame_id;
    box.header.stamp = traj_pub_node->now();
    box.type = 1;
    box.id = 0; //number for id later
    box.action = 0;
    box.pose = box_pose;
    box.scale.x = primitive.dimensions[primitive.BOX_X];
    box.scale.y = primitive.dimensions[primitive.BOX_Y];
    box.scale.z = primitive.dimensions[primitive.BOX_Z];
    box.color.r = 0.95;
    box.color.g = 0.95;
    box.color.b = 0.35;
    box.color.a = 1.0;
    marker_pub->publish(box);

    if(!planning_scene.processCollisionObjectMsg(collision_object))
    {
        RCLCPP_WARN_STREAM(LOGGER, "Object not able to be added"<<"\n");
    }

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    collision_request.contacts = true;
    collision_request.max_contacts = 1000;

    const std::string PLANNING_GROUP = "manipulator";
    moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    moveit::core::RobotState potential_state = current_state;

    const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup(PLANNING_GROUP);

    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.3;
    target_pose.position.y = 0.3;
    target_pose.position.z = 0.7;

    auto time_point1 = std::chrono::high_resolution_clock::now();

    while(rclcpp::ok())
    {
        //TODO: Add error magnitude checking. Should only work if the last generated solution is collision
        if(new_joint_state)
        {   
            //Reset flag
            new_joint_state=false;
            //Update current Joint State
            current_state.setJointGroupPositions(joint_model_group,positions);
            if(!current_state.setFromIK(joint_model_group,target_pose,"end_effector_link",0.0))
            {
                RCLCPP_WARN_STREAM(LOGGER, "IK not found"<<"\n");
                // marker_pub->publish(box);
            }
            else
            {
            // time_point1 = std::chrono::high_resolution_clock::now();
            current_state.copyJointGroupPositions(joint_model_group,desired_state);
            collision_result.clear();
            potential_state.setJointGroupPositions(joint_model_group,desired_state);
            planning_scene.checkCollision(collision_request, collision_result, potential_state);
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()-time_point1);
            RCLCPP_INFO_STREAM(LOGGER, "Collision detection time: "<< elapsed.count() <<"\n");

            if(collision_result.collision)
            {
                RCLCPP_WARN_STREAM(LOGGER, "The robot IK solution would cause a collision"<<"\n");
                collision_detection::CollisionResult::ContactMap::const_iterator it;
                for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
                {
                    RCLCPP_INFO(LOGGER, "Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
                }
            }
            std::copy(desired_state.begin(), desired_state.end(), desired_q.data());
            // desired_q = vectorFMod(desired_q);
            // RCLCPP_INFO_STREAM(LOGGER,"Target_position: \n" << desired_q.transpose() << "\n");
            // Eigen::Isometry3d end_effector_state = current_state.getGlobalLinkTransform("half_arm_2_link");
            // RCLCPP_INFO_STREAM(LOGGER,"Translation: \n" << end_effector_state.translation()<<"\n");
            // RCLCPP_INFO_STREAM(LOGGER,"Rotation: \n" << end_effector_state.rotation()<<"\n");
            }
        
            if(traj_gen_needed)
            {
                //Reset flag
                traj_gen_needed = false;
                
            }
        }
    }
    rclcpp::shutdown();
    return 0;
}