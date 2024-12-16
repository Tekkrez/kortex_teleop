#include <rclcpp/rclcpp.hpp>
#include <robot_util.h>
#include <math.h>
#include <chrono>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
//TODO: Get rid of this
#include <std_msgs/msg/float64_multi_array.hpp>
static const rclcpp::Logger LOGGER = rclcpp::get_logger("gen3_control_node");
std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7", "robotiq_85_right_knuckle_joint",
"robotiq_85_left_inner_knuckle_joint","robotiq_85_right_inner_knuckle_joint" ,"robotiq_85_left_finger_tip_joint","robotiq_85_right_finger_tip_joint","robotiq_85_left_knuckle_joint"};
//Loop rate var
int rate = 1100;
double period=1/static_cast<double>(rate);
//Variables for trajectory tracking
Eigen::VectorXd joint_vel_target;
bool new_velocity = false;
bool vel_target_reached = true;
int traj_position = 0;
//Soft accel limit
double soft_accel_limit = 60;
//Need to adjust the rest
Eigen::Matrix<double,1,7> next_joint_vel;
Eigen::VectorXd time_slices;
Eigen::Matrix<double,2,7> linear_func;
Eigen::VectorXd vel_command(7);

std::vector<int> continuous_joints = {0,2,4,6};
std::vector<int> non_continuous_joints = {1,3,5};
//joint_limits in degrees
std::vector<double> joint_limits = {0,138,0,152,0,127,0};

void traj_callback(const trajectory_msgs::msg::JointTrajectoryPoint& msg)
{
  joint_vel_target = radiansToDegrees(stdVecToEigen(msg.velocities));
  new_velocity = true;
}

int main(int argc, char** argv)
{
  // Initialize nodes and load parameters
  rclcpp::init(argc, argv);
  // rclcpp::NodeOptions node_options;
  // node_options.automatically_declare_parameters_from_overrides(true);
  auto gen3_control_node = rclcpp::Node::make_shared("gen3_control_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(gen3_control_node);
  std::thread([&executor]() { executor.spin(); }).detach();
  // Params
  gen3_control_node->declare_parameter("pos_pub_rate",120);
  // gen3_control_node->declare_parameter("enable_send_position",true);
  // gen3_control_node->declare_parameter("pub_test_positions",false);
  int pos_pub_rate = gen3_control_node->get_parameter("pos_pub_rate").as_int();
  // bool enable_send_position = gen3_control_node->get_parameter("enable_send_position").as_bool();
  // bool pub_test_positions = gen3_control_node->get_parameter("pub_test_positions").as_bool();
  //Joint state publi@id:ms-vscode.cpptools-extension-packsher
  rclcpp::QoS pub_qos(1);
  pub_qos.best_effort();
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
  joint_pub = gen3_control_node->create_publisher<sensor_msgs::msg::JointState>("joint_states",pub_qos);
  //Trajectory sub
  rclcpp::QoS sub_qos(1);
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr traj_sub;
  traj_sub = gen3_control_node->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>("joint_trajectory",sub_qos,traj_callback);
  
  //Test pub
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pos_test_pub;
  joint_pos_test_pub = gen3_control_node->create_publisher<std_msgs::msg::Float64MultiArray>("joint_pos_test",pub_qos);
  int pos_pub_period = hzToLoopNum(pos_pub_rate,rate);
  rclcpp::Rate loop_rate(rate);
  int i=0;
  Eigen::VectorXd gen3_q_sim(7);
  gen3_q_sim <<0,15,180,-130,0,55,90;
  Eigen::VectorXd gen3_q_dot_sim(7);
  gen3_q_dot_sim << 0,0,0,0,0,0,0;
  Eigen::VectorXd gen3_q_dotdot_sim(7);
  gen3_q_dotdot_sim << 0,0,0,0,0,0,0;
  Eigen::VectorXd prev_q_dot = gen3_q_dot_sim;

  LowPassFilter accel_filt;
  accel_filt.lowPassFilterInit(gen3_q_dotdot_sim,1.0);

  while(rclcpp::ok())
  {
    i++;
    //Check for new message and smoothly vary velocity (may not be neccesary)
    if(new_velocity)
    {
      // Reset flags and position counter
      new_velocity=false;
      vel_target_reached = false;
      traj_position = 0;
      double completion_time = (joint_vel_target-gen3_q_dot_sim).cwiseAbs().maxCoeff()/soft_accel_limit;
      if(completion_time < 2*period){
        vel_target_reached = true;
        vel_command = joint_vel_target;
      }
      else
      {
        time_slices = Eigen::VectorXd::LinSpaced(completion_time*rate, period, completion_time);
        linear_func = findLinearFunction(gen3_q_dot_sim,joint_vel_target,completion_time);
      }
    } 
    //Get to target velocity for timestep
    //TODO: Need timing mechanism to ensure it doesn't spin forever. Should tie it in with completion time and collision detection
    if(!vel_target_reached)
    {
      //Get desired Joint vales at time slice
      //Get velocity
      Eigen::Matrix<double,1,2> time_matrix;
      // std::cout<<"Time slice: "<< traj_position << " out of " << time_slices.size() << std::endl;
      time_matrix.row(0) << 1,time_slices(traj_position);
      // time_matrix.row(0) << 1,time_slices(traj_position),pow(time_slices(traj_position),2),pow(time_slices(traj_position),3);
      // time_matrix.row(1) << 0,1,2*time_slices(traj_position),3*pow(time_slices(traj_position),2);
      Eigen::Matrix<double,1,7> result;
      result = time_matrix*linear_func;
      // vel_command = result.row(0).transpose();
      vel_command = result;
    }
    //Simulate command
    //Update sim position assuming perfect tracking
    //Go through continuous joints
    Eigen::VectorXd delta_pos = vel_command * period; 
    for(auto & joint : continuous_joints)
    {
        gen3_q_sim(joint)=fmod(gen3_q_sim(joint)+delta_pos(joint),360.0);
    }

    // Non continuous joints
    for(auto & joint : non_continuous_joints)
    {
      if(abs(gen3_q_sim(joint)+delta_pos(joint))>joint_limits[joint])
      {
        std::cout<<"JOINT LIMIT EXECEDED FOR JOINT " << joint << "Requesting: "<< gen3_q_sim(joint)+delta_pos(joint) <<" with lim: "<< joint_limits[joint]<< std::endl;
        vel_command(joint) = 0;
      }
      else
      {
        gen3_q_sim(joint)=fmod(gen3_q_sim(joint)+delta_pos(joint),360.0);
      }
    }
    gen3_q_dot_sim = vel_command;
    gen3_q_dotdot_sim = accel_filt.applyFilter((gen3_q_dot_sim-prev_q_dot)/(period));
    prev_q_dot = gen3_q_dot_sim;
    if(!vel_target_reached){
      traj_position++;
    }
    if(traj_position == time_slices.size())
    {
      vel_target_reached = true;
    }
    if(!(i%pos_pub_period))
    {
      // std::cout << "delta_pos" << vel_command << std::endl;
      // std::cout << "delta_pos" << delta_pos << std::endl;
      auto message = sensor_msgs::msg::JointState();
      //Fill in message
      message.header.stamp = gen3_control_node->now();
      message.name = joint_names;
      
      std::vector<double> out_pos = eigenToStdVec(degreesToRadians(gen3_q_sim));
      std::vector<double> out_vel = eigenToStdVec(degreesToRadians(gen3_q_dot_sim));
      std::vector<double> out_acc = eigenToStdVec(degreesToRadians(gen3_q_dotdot_sim));
      out_pos.resize(13);
      out_vel.resize(13);
      out_acc.resize(13);
      message.position = out_pos;
      message.velocity = out_vel;
      message.effort = out_acc;
      //Publish
      joint_pub->publish(message);
    }

    loop_rate.sleep(); 
  }

  rclcpp::shutdown();
  return 0;
}