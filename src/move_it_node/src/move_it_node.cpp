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
#include <trajectory_msgs/msg/joint_trajectory_point.h>
// Util
#include <robot_util.h>
#include <chrono>
#include <tf2_eigen/tf2_eigen.hpp>
using std::placeholders::_1;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_traj_node");

//Joint names
std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7" };
std::vector<int> continuous_joints = {0,2,4,6};
std::vector<int> non_continuous_joints = {1,3,5};
Eigen::VectorXd latest_q(7);
Eigen::VectorXd latest_q_dot(7);

//Targets
geometry_msgs::msg::Pose target_pose;
std::vector<double> desired_state;
Eigen::VectorXd desired_q(7);
Eigen::Isometry3d prevPose;

//Params
double soft_joint_speed_limit = 0.2; //rad/sec
double time_step = 0.1;
double linear_distance_thresh = 0.005;
double angular_distance_thresh = 0.01;

// Flags
bool new_target_pose = false;
bool traj_gen_needed = false;
bool new_joint_state = false;
bool doOnce = true;

void jointState_Callback(const sensor_msgs::msg::JointState& msg)
{

    new_joint_state = true;
    std::vector<double> positions;
    std::vector<double> velocities;

    positions = msg.position;
    velocities = msg.velocity;

    std::copy(positions.begin(),positions.end(),latest_q.data());
    std::copy(velocities.begin(),velocities.end(),latest_q_dot.data());
}
void targetPose_callback(const geometry_msgs::msg::Pose& msg)
{
    //Check if diffrent from previous target pose
    Eigen::Isometry3d desPose;
    tf2::fromMsg(msg,desPose);
    double const linear_distance = (desPose.translation()-prevPose.translation()).norm();
    auto const q_1 = Eigen::Quaterniond(desPose.rotation());
    auto const q_2 = Eigen::Quaterniond(prevPose.rotation());
    double const angular_distance = q_2.angularDistance(q_1);
    std::cout<<"New Pose Received: "<<std::endl;
    //Update target pose if different enough
    if(linear_distance>linear_distance_thresh || std::abs(angular_distance)>angular_distance_thresh)
    {
        std::cout<<"New Pose is different enough: "<<std::endl;
        target_pose = msg;
        prevPose = desPose;
        new_target_pose = true;
    }
}

Eigen::VectorXd targetAdjustmentContinuousJoints(Eigen::VectorXd& currentPos, Eigen::VectorXd& targetPos)
{
    Eigen::VectorXd adjustedTarget = targetPos;
    Eigen::VectorXd diff = targetPos-currentPos;
    std::cout << "Diff: " << diff.transpose() << std::endl;
    for(int i : continuous_joints)
    {
        if(abs(diff(i))>M_PI)
        {
            RCLCPP_WARN_STREAM(LOGGER, "Large desired diff in continuous joint. Look at Joint: "<<i+1<<"\n");

            if(diff(i)<0)
            {
                adjustedTarget(i) = adjustedTarget(i) + 2*M_PI;
            }
            else
            {
                adjustedTarget(i) = adjustedTarget(i) - 2*M_PI;
            }
        }
    }
    std::cout << "Adjusted Target: " << adjustedTarget.transpose() << std::endl;
    return adjustedTarget;
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
    //Subscribers
    rclcpp::QoS sub_qos(1);
    sub_qos.best_effort();
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointState_sub;
    jointState_sub = traj_pub_node->create_subscription<sensor_msgs::msg::JointState>("joint_states",sub_qos,jointState_Callback);
    //Desired Pose sub
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub;
    pose_sub= traj_pub_node->create_subscription<geometry_msgs::msg::Pose>("desired_pose",sub_qos,targetPose_callback);
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
    box_pose.position.z = -0.14;//3 cm extra offset since robot is sitting on two plates
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

    auto time_point1 = std::chrono::high_resolution_clock::now();

    while(rclcpp::ok())
    {
        //TODO: Add error magnitude checking. Should only work if the last generated solution is collision
        if(new_target_pose && new_joint_state)
        {   
            //Reset flag
            new_target_pose = false;
            new_joint_state = false;
            //Update current Joint State
            current_state.setJointGroupPositions(joint_model_group,latest_q);
            if(!current_state.setFromIK(joint_model_group,target_pose,"end_effector_link",0.0))
            {
                RCLCPP_WARN_STREAM(LOGGER, "IK not found"<<"\n");
                //redo the traj gen process with the hope of getting one that works
                new_target_pose = true;
            }
            else
            {
            // time_point1 = std::chrono::high_resolution_clock::now();
            current_state.copyJointGroupPositions(joint_model_group,desired_state);
            collision_result.clear();
            potential_state.setJointGroupPositions(joint_model_group,desired_state);
            planning_scene.checkCollision(collision_request, collision_result, potential_state);
            // auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()-time_point1);
            // RCLCPP_INFO_STREAM(LOGGER, "Collision detection time: "<< elapsed.count() <<"\n");

            if(collision_result.collision)
            {
                RCLCPP_WARN_STREAM(LOGGER, "The robot IK solution would cause a collision"<<"\n");
                collision_detection::CollisionResult::ContactMap::const_iterator it;
                for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
                {
                    RCLCPP_INFO(LOGGER, "Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
                }
                //redo the traj gen process with the hope of getting one that works
                new_target_pose = true;
            }
            else
            {
                std::copy(desired_state.begin(), desired_state.end(), desired_q.data());
                traj_gen_needed = true;
            }
            // desired_q = vectorFMod(desired_q);
            RCLCPP_INFO_STREAM(LOGGER,"Target_position: " << desired_q.transpose() << "\n");
            // Eigen::Isometry3d end_effector_state = current_state.getGlobalLinkTransform("half_arm_2_link");
            // RCLCPP_INFO_STREAM(LOGGER,"Translation: \n" << end_effector_state.translation()<<"\n");
            // RCLCPP_INFO_STREAM(LOGGER,"Rotation: \n" << end_effector_state.rotation()<<"\n");
            }
        
            if(traj_gen_needed)
            {
                //Reset flag
                traj_gen_needed = false;
                Eigen::Matrix <double,4,7> cubicFunc;
                Eigen::VectorXd adjusted_target = targetAdjustmentContinuousJoints(latest_q,desired_q);
                Eigen::VectorXd diff = adjusted_target-latest_q;
                double completion_time = diff.cwiseAbs().maxCoeff()/soft_joint_speed_limit;
                completion_time = round(completion_time/time_step)*time_step;
                cubicFunc = findCubicFunction(latest_q, adjusted_target, latest_q_dot, completion_time);
                RCLCPP_INFO_STREAM(LOGGER, "Cubic Function:\n" << cubicFunc << "\n");

                //TODO: Add checking for max time_slice required
                Eigen::VectorXd time_slices = Eigen::VectorXd::LinSpaced(completion_time/time_step,time_step,completion_time);
                
                for (int i=0; i<time_slices.size();i++)
                {
                    Eigen::Matrix<double,2,4> time_matrix;
                    time_matrix.row(0) <<  1,time_slices(i),pow(time_slices(i),2),pow(time_slices(i),3);
                    time_matrix.row(1) <<  0,1,2*time_slices(i),3*pow(time_slices(i),2);

                    //Desired Joint vales at time slice
                    Eigen::Matrix<double,2,7> result;
                    result = time_matrix*cubicFunc;
                    //Check collision
                    RCLCPP_INFO_STREAM(LOGGER, "Waypoint: "<< i <<" at Position: " << result << "\n");

                    potential_state.setJointGroupPositions(joint_model_group,result.row(0));
                    collision_result.clear();
                    planning_scene.checkCollision(collision_request, collision_result, potential_state);
                    if(collision_result.collision)
                    {
                        RCLCPP_WARN_STREAM(LOGGER, "The robot IK trajectory would result in a collision"<<"\n");
                        collision_detection::CollisionResult::ContactMap::const_iterator it;
                        for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
                        {
                            RCLCPP_INFO(LOGGER, "Contact between: %s and %s at time slice %d", it->first.first.c_str(), it->first.second.c_str(),i+1);
                        }
                        //redo the traj gen process with the hope of getting one that works
                        new_target_pose = true;
                        break;
                    }
                }

                if(!collision_result.collision)
                {
                    //pub joint_trajectory
                    joint_traj_pub->publish(fillJointTrajectoryPoint(adjusted_target, Eigen::VectorXd::Zero(7), completion_time));
                }
            }
        }
    }
    rclcpp::shutdown();
    return 0;
}