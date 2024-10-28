#include <rclcpp/rclcpp.hpp>
#include <robot_util.h>
#include <math.h>
#include <chrono>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
static const rclcpp::Logger LOGGER = rclcpp::get_logger("gen3_control_node");
std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7", "robotiq_85_right_knuckle_joint",
"robotiq_85_left_inner_knuckle_joint","robotiq_85_right_inner_knuckle_joint" ,"robotiq_85_left_finger_tip_joint","robotiq_85_right_finger_tip_joint","robotiq_85_left_knuckle_joint"};
//Loop rate var
int rate = 1100;
//Variables for trajectory tracking
Eigen::VectorXd joint_pos_target;
bool new_trajectory = false;
bool traj_end_reached = true;
int traj_position = 0;
Eigen::Matrix<double,1,7> next_joint_step;
double target_time;
Eigen::VectorXd time_slices;
Eigen::Matrix<double,4,7> cubicFunc;

void traj_callback(const trajectory_msgs::msg::JointTrajectoryPoint& msg)
{
  rclcpp::Duration time_to_first_point(msg.time_from_start.sec,msg.time_from_start.nanosec);
  target_time = time_to_first_point.seconds();
  joint_pos_target = radiansToDegrees(stdVecToEigen(msg.positions));
  new_trajectory = true;
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
  gen3_control_node->declare_parameter("pos_pub_rate",60);
  // gen3_control_node->declare_parameter("enable_send_position",true);
  // gen3_control_node->declare_parameter("pub_test_positions",false);
  int pos_pub_rate = gen3_control_node->get_parameter("pos_pub_rate").as_int();
  // bool enable_send_position = gen3_control_node->get_parameter("enable_send_position").as_bool();
  // bool pub_test_positions = gen3_control_node->get_parameter("pub_test_positions").as_bool();
  //Joint state publisher
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

  while(rclcpp::ok())
  {
    i++;
    //Check for new message
    if(new_trajectory)
    {
    //RCLCPP_WARN_STREAM(LOGGER, "Here 1"<<"\n");
      new_trajectory=false;
      traj_end_reached = false;
      traj_position = 0;
      time_slices = Eigen::VectorXd::LinSpaced(target_time*rate, 1/static_cast<double>(rate), target_time);
      cubicFunc = findCubicFunction(gen3_q_sim,joint_pos_target,gen3_q_dot_sim,target_time);
    }
    //Get next joint position for timestep
    if(!traj_end_reached)
    {
      //Get desired Joint vales at time slice
      //Get velocity
      Eigen::Matrix<double,2,4> time_matrix;
      time_matrix.row(0) <<  1,time_slices(traj_position),pow(time_slices(traj_position),2),pow(time_slices(traj_position),3);
      time_matrix.row(1) << 0,1,2*time_slices(traj_position),3*pow(time_slices(traj_position),2);
      Eigen::Matrix<double,2,7> result;
      result = time_matrix*cubicFunc;
      // std::cout<<"Result: " << result.row(0) << " Result vel :" << result.row(1) <<"\n";
      std::vector<double> pos_command;
      pos_command = eigenToStdVec(result.row(0).transpose());
      for(int i =0;i<gen3_q_sim.size();i++)
      {
        pos_command[i] = fmod(pos_command[i],360.0);
        if(pos_command[i]>180)
        {
          pos_command[i] = pos_command[i]-360;
        }
      }
      //Update sim position assuming perfect tracking
      gen3_q_sim = stdVecToEigen(pos_command);
      gen3_q_dot_sim = result.row(1).transpose();
      //Check if end of time slice
      traj_position++;
      if(traj_position == time_slices.size())
      {
        traj_end_reached = true;
      }
    }
    if(!(i%pos_pub_period))
    {
      //TODO: Get rid of this check later
      auto message = sensor_msgs::msg::JointState();
      //Fill in message
      message.header.stamp = gen3_control_node->now();
      message.name = joint_names;
      
      std::vector<double> out_pos = eigenToStdVec(degreesToRadians(gen3_q_sim));
      std::vector<double> out_vel = eigenToStdVec(degreesToRadians(gen3_q_dot_sim));
      out_pos.resize(13);
      out_vel.resize(13);
      message.position = out_pos;
      message.velocity = out_vel;
      //Publish
      joint_pub->publish(message);
      } 

    loop_rate.sleep(); 
  }

  rclcpp::shutdown();
  return 0;
}