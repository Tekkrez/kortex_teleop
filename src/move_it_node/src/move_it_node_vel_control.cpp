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
#include <std_msgs/msg/float64_multi_array.hpp>
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
Eigen::Isometry3d potential_target_pose;
Eigen::Isometry3d target_pose;
//params
double linear_distance_thresh, angular_distance_thresh, joint_lim_bound, inv_vel_scale, sec_task_vel_scale, max_joint_vel;
double cc_max_time, cc_resolution, cc_halt_time, tanh_x_scaling, tanh_x_translation;
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

double timeTillCollision(const Eigen::VectorXd& jointVelTarget,const Eigen::VectorXd& currentPos, const moveit::core::JointModelGroup* jointModelGroup, moveit::core::RobotState& potentialState, const planning_scene::PlanningScene& planningScene)
{
    //Variables for collision detection
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;
    collision_request.max_contacts = 10;
    //Timeslice to extrapolate joint velocities with
    auto time_slices = Eigen::VectorXd::LinSpaced(cc_resolution,0,cc_max_time);
    // //Iterate thorugh the time slices and check if extrapolated joint states will result in a collision
    for(double t : time_slices)
    {
        potentialState.setJointGroupPositions(jointModelGroup,currentPos+jointVelTarget*t);
        collision_result.clear();
        planningScene.checkCollision(collision_request, collision_result, potentialState);
        if(collision_result.collision)
        {
            // RCLCPP_WARN_STREAM(LOGGER, "Joint velocities are in collision range"<<"\n");
            RCLCPP_INFO_STREAM(LOGGER, "Time slice: "<< t <<"/" << cc_max_time <<"\n");
            collision_detection::CollisionResult::ContactMap::const_iterator it;
            for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
            {
                // RCLCPP_INFO(LOGGER, "Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
            }
            return t;
        }
    }
    return 100;
}


int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto traj_pub_node = rclcpp::Node::make_shared("traj_pub_node", node_options);
    //Params
    linear_distance_thresh = traj_pub_node->get_parameter("linear_distance_thresh").as_double();
    angular_distance_thresh = traj_pub_node->get_parameter("angular_distance_thresh").as_double();
    joint_lim_bound = traj_pub_node->get_parameter("joint_limit_bound").as_double(); //rad/sec
    inv_vel_scale = traj_pub_node->get_parameter("inverse_velocity_scaling").as_double();
    sec_task_vel_scale = traj_pub_node->get_parameter("secondary_task_velocity_scaling").as_double();
    max_joint_vel = traj_pub_node->get_parameter("max_joint_velocity").as_double();
    cc_max_time = traj_pub_node->get_parameter("collision_check_max_time").as_double();
    cc_resolution = traj_pub_node->get_parameter("collision_check_resolution").as_double();
    cc_halt_time = traj_pub_node->get_parameter("collision_check_halt_time").as_double();
    tanh_x_scaling = traj_pub_node->get_parameter("tanh_horizontal_scaling").as_double();
    tanh_x_translation = traj_pub_node->get_parameter("tanh_horizontal_translation").as_double();
    
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
    //Test pub
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pos_test_pub;
    joint_pos_test_pub = traj_pub_node->create_publisher<std_msgs::msg::Float64MultiArray>("twists",1);
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pos_test2_pub;
    joint_pos_test2_pub = traj_pub_node->create_publisher<std_msgs::msg::Float64MultiArray>("gradients_vel_contribution",1);

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

    const std::string PLANNING_GROUP = "manipulator";
    moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    moveit::core::RobotState potential_state = current_state;
    const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup(PLANNING_GROUP);
      

    while(rclcpp::ok())
    {
        if(new_joint_state && target_pose_received)
        {
            //Reset flag
            new_joint_state = false;
            //Update current Joint State and potential state
            current_state.setJointGroupPositions(joint_model_group,latest_q);

            //Get pose error values
            const Eigen::Isometry3d current_pose = current_state.getGlobalLinkTransform("end_effector_link");
            const Eigen::Quaterniond current_orientation(current_pose.rotation());
            const Eigen::Quaterniond target_orientation(target_pose.rotation());
            const Eigen::AngleAxisd orientation_error(target_orientation*current_orientation.inverse());
            const Eigen::Vector3d position_error(target_pose.translation()-current_pose.translation());
            Eigen::Vector3d omega;

            //Update output velocity if different enough
            if(position_error.norm()>linear_distance_thresh || std::abs(orientation_error.angle())>angular_distance_thresh)
            {
                //Assert act as a check, likely uneccesary since Eigen should take care of this when creating the angle axis object
                assert(abs(orientation_error.angle())<=M_PI);
                omega = orientation_error.angle()*orientation_error.axis();
                //Find twists from pose errors
                Eigen::VectorXd twists(6);
                twists << (target_pose.translation()-current_pose.translation()),omega;
                //Jacobians
                const Eigen::MatrixXd jacobian = current_state.getJacobian(joint_model_group);
                const Eigen::MatrixXd jacobian_pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
                //Perhaps use dls Jacobian for better singularity avoidance. Comes with the need to recalculate lambda for optimal usage
                //GPM for joint limit management, which is only relevant for the non continuous joints
                //This secondary task only activates once past the upper or lower bound.
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

                auto test_msg1 = std_msgs::msg::Float64MultiArray();
                test_msg1.data = eigenToStdVec(twists);
                joint_pos_test_pub->publish(test_msg1);
                auto test_msg2 = std_msgs::msg::Float64MultiArray();
                test_msg2.data = eigenToStdVec((Eigen::MatrixXd::Identity(7,7)-jacobian_pinv*jacobian)*sec_task_gradients);
                joint_pos_test2_pub->publish(test_msg2);

                Eigen::VectorXd joint_velocities = inv_vel_scale*jacobian_pinv*twists + sec_task_vel_scale*(Eigen::MatrixXd::Identity(7,7)-jacobian_pinv*jacobian)*sec_task_gradients;
                
                //Scale velocities
                if(joint_velocities.cwiseAbs().maxCoeff()>max_joint_vel)
                {
                    const double vel_scaling = (joint_velocities/max_joint_vel).cwiseAbs().maxCoeff();
                    joint_velocities = joint_velocities/vel_scaling;
                }
                // auto timepoint = std::chrono::high_resolution_clock::now();
                const double collision_time = timeTillCollision(joint_velocities,latest_q,joint_model_group, potential_state, planning_scene);
                // auto delta = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()-timepoint);
                // std::cout<< "Collision check time: " << delta.count() << std::endl;
                if(collision_time<=cc_halt_time)
                {
                    std::cout<< "Stopped for collision************************************" << std::endl;
                    joint_traj_pub->publish(fillJointTrajectoryPoint(Eigen::VectorXd::Zero(7),Eigen::VectorXd::Zero(7), 1.0));
                }
                else
                {
                    if(collision_time<100)
                    {
                        //Scale joint_velocities based on time to collide
                        //tanh func is 0.2 at t=0.2, 0.918 at t=0.75
                        joint_velocities *= std::tanh(tanh_x_scaling*collision_time-tanh_x_translation);
                    }
                    // pub joint_trajectory
                    joint_traj_pub->publish(fillJointTrajectoryPoint(Eigen::VectorXd::Zero(7),joint_velocities, 1.0));
                }
            }
            else
            {
                joint_traj_pub->publish(fillJointTrajectoryPoint(Eigen::VectorXd::Zero(7),Eigen::VectorXd::Zero(7), 1.0));
            }
        }
    }
    rclcpp::shutdown();
    return 0;
}