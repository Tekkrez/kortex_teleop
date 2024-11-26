//ROS
#include <rclcpp/rclcpp.hpp>
//Basic Move it stuff
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
//For goal pose?
#include <moveit/kinematic_constraints/utils.h>
//MSGs
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.h>
// Util
#include <robot_util.h>
#include <chrono>
#include <thread>
#include <tf2_eigen/tf2_eigen.hpp>
using std::placeholders::_1;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_traj_node");

//Joint names
std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7" };
std::vector<int> continuous_joints = {0,2,4,6};
std::vector<int> non_continuous_joints = {1,3,5};
//Other two joint limits are larger than this, but keep it here for simplicity and additional room for error
double joint_lim = 2.23;
Eigen::VectorXd latest_q(7);
Eigen::VectorXd latest_q_dot(7);
Eigen::VectorXd latest_q_dotdot(7);

//Targets
std::vector<double> desired_state;
Eigen::VectorXd desired_q(7);
Eigen::Isometry3d potential_target_pose;
Eigen::Isometry3d target_pose;
//params
double soft_joint_speed_limit,time_step,linear_distance_thresh,angular_distance_thresh;
double starting_displacement_weight,large_delta_thresh,large_delta_weight,medium_delta_thresh,medium_delta_weight;
// Flags
bool target_pose_received = false;
bool traj_gen_needed = false;
bool new_joint_state = false;
bool large_delta_pose = false;
bool medium_delta_pose = false;

void jointState_Callback(const sensor_msgs::msg::JointState& msg)
{
    new_joint_state = true;
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> accelerations;

    positions = msg.position;
    velocities = msg.velocity;
    accelerations = msg.effort;

    std::copy(positions.begin(),positions.begin()+7,latest_q.data());
    std::copy(velocities.begin(),velocities.begin()+7,latest_q_dot.data());
    std::copy(accelerations.begin(),accelerations.begin()+7,latest_q_dotdot.data());
}

void targetPose_callback(const geometry_msgs::msg::PoseStamped& msg)
{
    //Copy message to target_pose
    tf2::fromMsg(msg.pose,target_pose);
    target_pose_received = true;
}

trajectory_msgs::msg::JointTrajectoryPoint fillJointTrajectoryPoint(Eigen::VectorXd positions, Eigen::VectorXd velocities, double timeVal)
{
  //Convert from Eigen vec to std vec
  std::vector<double> position_vec;
  position_vec.resize(positions.size());
  std::vector<double> velocity_vec;
  velocity_vec.resize(velocities.size());
  Eigen::VectorXd::Map(&position_vec[0],position_vec.size()) = positions;
  Eigen::VectorXd::Map(&velocity_vec[0],velocity_vec.size()) = velocities;
  //Get time to complete
  int32_t seconds = (int32_t)(timeVal);
  int32_t nanoseconds = (int32_t)((timeVal-seconds)*1e9);
  rclcpp::Duration waypoint_time(seconds,nanoseconds);
  //Fill in joint traj point
  trajectory_msgs::msg::JointTrajectoryPoint waypoint;
  waypoint.positions = position_vec;
  waypoint.velocities = velocity_vec;
  waypoint.time_from_start = waypoint_time;

  return waypoint;
}


int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto traj_pub_node = rclcpp::Node::make_shared("traj_pub_node", node_options);
    //Params
    soft_joint_speed_limit = traj_pub_node->get_parameter("soft_joint_speed_limit").as_double(); //rad/sec
    time_step = traj_pub_node->get_parameter("time_step").as_double();
    linear_distance_thresh = traj_pub_node->get_parameter("linear_distance_thresh").as_double();
    angular_distance_thresh = traj_pub_node->get_parameter("angular_distance_thresh").as_double();
    starting_displacement_weight = traj_pub_node->get_parameter("robot_description_kinematics.manipulator.minimal_displacement_weight").as_double();
    large_delta_thresh = traj_pub_node->get_parameter("large_delta_thresh").as_double();
    large_delta_weight = traj_pub_node->get_parameter("large_delta_weight").as_double();
    medium_delta_thresh = traj_pub_node->get_parameter("medium_delta_thresh").as_double();
    medium_delta_weight = traj_pub_node->get_parameter("medium_delta_weight").as_double();
    
    //Subscribers
    rclcpp::QoS sub_qos(1);
    sub_qos.best_effort();
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointState_sub;
    jointState_sub = traj_pub_node->create_subscription<sensor_msgs::msg::JointState>("joint_states",sub_qos,jointState_Callback);
    //Desired Pose sub
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    pose_sub= traj_pub_node->create_subscription<geometry_msgs::msg::PoseStamped>("desired_pose",sub_qos,targetPose_callback);
    //Rviz2 marker pub
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    marker_pub = traj_pub_node->create_publisher<visualization_msgs::msg::Marker>("visualization_marker",1);
    //Joint trajectory pub
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr joint_traj_pub;
    joint_traj_pub = traj_pub_node->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("joint_trajectory",1);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(traj_pub_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    //Load move it stuff
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
    box_pose.position.z = -0.13;//3.5 cm extra offset since robot is sitting on two plates
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    //rviz vizualization
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
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    marker_pub->publish(box);

    //Add collision object to planning scecne
    if(!planning_scene.processCollisionObjectMsg(collision_object))
    {
        RCLCPP_WARN_STREAM(LOGGER, "Object not able to be added"<<"\n");
    }

    //Variables for collision detection
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;
    collision_request.max_contacts = 10;

    const std::string PLANNING_GROUP = "manipulator";
    moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    moveit::core::RobotState potential_state = current_state;

    const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup(PLANNING_GROUP);

    // auto time_point1 = std::chrono::high_resolution_clock::now();

    while(rclcpp::ok())
    {
        if(new_joint_state && target_pose_received)
        {
            //Reset flag
            new_joint_state = false;
            //Update current Joint State and potential state
            current_state.setJointGroupPositions(joint_model_group,latest_q);
            Eigen::Isometry3d current_pose = current_state.getGlobalLinkTransform("end_effector_link");
            //Determine if new pose is different enough from current pose
            Eigen::Isometry3d delta_pose = current_pose.inverse()*target_pose;
            Eigen::AngleAxisd angle_axis(delta_pose.rotation());
            Eigen::Vector3d omega;
            //Update output velocity if different enough
            if(delta_pose.translation().norm()>linear_distance_thresh || std::abs(angle_axis.angle())>angular_distance_thresh)
            {
                potential_state.setJointGroupPositions(joint_model_group,latest_q);
                if(abs(angle_axis.angle())<M_PI)
                {
                    omega = angle_axis.angle()*angle_axis.axis();
                }
                else if(angle_axis.angle()<=-M_PI)
                {
                    omega = (2*M_PI+angle_axis.angle())*angle_axis.axis();
                }
                else if(angle_axis.angle()>=M_PI)
                {
                    omega = (-2*M_PI+angle_axis.angle())*angle_axis.axis();
                }
                Eigen::VectorXd twists(6);
                twists << delta_pose.translation(),omega;
                // Perhaps scale twists with diagonal matrix
                Eigen::MatrixXd jacobian = current_state.getJacobian(joint_model_group);
                Eigen::MatrixXd jacobian_pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
                //Perhaps use dls Jacobian for better singularity avoidance. Comes with the need to recalculate lambda for optimal usage
                //GPM for joint limit management, which is only relevant for the non continuous joints
                //This secondary task only activates once past the upper or lower bound.
                //TODO: Setup as param
                double joint_lim_bound = 1.5;
                Eigen::VectorXd sec_task_gradients(7);
                for(auto & joint : non_continuous_joints)
                {
                    if(latest_q(joint)>joint_lim_bound)
                    {
                        sec_task_gradients(joint) = -(latest_q(joint)-joint_lim_bound)/2*joint_lim;
                    }
                    else if(latest_q(joint)<-joint_lim_bound)
                    {
                        sec_task_gradients(joint) = -(latest_q(joint)+joint_lim_bound)/2*joint_lim;
                    }
                }
                Eigen::VectorXd joint_velocities = jacobian_pinv*twists + (Eigen::MatrixXd::Identity(7,7)-jacobian_pinv*jacobian)*sec_task_gradients;

                //pub joint_trajectory
                joint_traj_pub->publish(fillJointTrajectoryPoint(Eigen::VectorXd::Zero(7),joint_velocities, 1.0));
                //TODO: Check incremental collision and assosiciated completion time
            }
            //TODO: Possible edge cases opened up here
            else
            {
            }
            //
            //TODO: Scale \omega??
                // Handle the special case of zero rotation
                // if (omega.norm() < 1e-6) {
                //     omega.setZero();
                // } else {
                //     // Scale the rotation vector to ensure proper logarithm
                //     double theta = omega.norm();
                //     omega /= theta;
                //     omega *= std::sin(theta / 2) / (theta / 2);
                // }
        }
    }
    rclcpp::shutdown();
    return 0;
}