#include <robot_util.h>
#include <kortex_robot.h>
#include <rclcpp/rclcpp.hpp>
#include <math.h>
#include <chrono>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("gen3_control_node");
std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7" };
//Loop rate var
int rate = 1100;
//Variables for trajectory tracking
int waypoint_position = 0;
int waypoint_end = 0;
trajectory_msgs::msg::JointTrajectory trajectory;
bool new_trajectory = false;
bool trajectory_ready = false;
bool waypoint_reached = true;
double waypoint_interval = 0;
Eigen::Matrix<double,1,7> next_joint_step;

Eigen::VectorXd time_slices;
int time_slice_position = 0;
int time_slice_end = 0;
Eigen::Matrix<double,4,7> cubicFunc;

void traj_callback(const trajectory_msgs::msg::JointTrajectory& msg){
  trajectory = msg;
  waypoint_end = msg.points.size();
  rclcpp::Duration time_to_first_point(msg.points[0].time_from_start.sec,msg.points[0].time_from_start.nanosec);
  waypoint_interval = time_to_first_point.seconds();
  time_slices = Eigen::VectorXd::LinSpaced(waypoint_interval*rate, 1/static_cast<double>(rate), waypoint_interval);
  // std::cout<< "Time slices: " << time_slices.transpose() << std::endl;
  time_slice_end = time_slices.size()-1;
  new_trajectory = true;
  trajectory_ready = true;
}

int main(int argc, char** argv)
{
  // Initialize nodes and load parameters
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto gen3_control_node = rclcpp::Node::make_shared("gen3_control_node",node_options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(gen3_control_node);
  std::thread([&executor]() { executor.spin(); }).detach();
  //Joint state publisher
  rclcpp::QoS pub_qos(1);
  pub_qos.best_effort();
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
  joint_pub = gen3_control_node->create_publisher<sensor_msgs::msg::JointState>("joint_states",pub_qos);
  //Trajectory sub
  rclcpp::QoS sub_qos(1);
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub;
  traj_sub = gen3_control_node->create_subscription<trajectory_msgs::msg::JointTrajectory>("joint_trajectory",sub_qos,traj_callback);
  
  
  //Testing stuff
  Eigen::Matrix<double,4,7> cubicFunc;
  double timeEnd = 5.4;
  cubicFunc.row(0) << 2.57028e-05,0.261868,3.14159,-2.26902, -1.17179e-05,0.959858,1.57082;
  cubicFunc.row(1) << 0,0,0,0,0,0,0;
  cubicFunc.row(2) << 0.0488974,0.0106583,0.0441472,0.0572635,-0.0172877,0.111406,0.0633344;
  cubicFunc.row(3) << -0.00603672,-0.00131584,-0.00545027,-0.00706957,0.00213428,-0.0137538,-0.00781906;
  Eigen::VectorXd time_slices;
  time_slices = Eigen::VectorXd::LinSpaced(timeEnd*rate,1/static_cast<double>(rate),timeEnd);
  Eigen::MatrixXd joint_traj(time_slices.size(),7);
  // std::cout << "Time slices: " << time_slices.transpose() << std::endl;
  // std::cout<<"Cubic Function: "<<cubicFunc<< std::endl;
  for(int i=0;i<time_slices.size();i++)
  {
    Eigen::Matrix<double,1,4> time_matrix;
    time_matrix << 1,time_slices(i),pow(time_slices(i),2),pow(time_slices(i),3);
    Eigen::Matrix<double,1,7> result;
    // std::cout<<"Time_matrix: "<<time_matrix<< std::endl;
    result = time_matrix*cubicFunc;
    // std::cout<<"Calculated this angle for joint "<< 7 <<": "<< time_matrix*cubicFunc.col(6) << std::endl;
    result = radiansToDegrees(time_matrix*cubicFunc);
    joint_traj.row(i) = result;
  }
  auto gen3_robot = kortex_robot();
  gen3_robot.setLowLevelServoing();
  gen3_robot.getFeedback();
  gen3_robot.setBaseCommand();
  rclcpp::Rate loop_rate(rate);
  int i=0;
  int j=0;
  while(rclcpp::ok())
  {
    i++;
    //Send joint step
    if(i==4999)
    {
      std::cout<<"About to start sending positions"<< std::endl;

    }
    if(i>5000 && j<time_slices.size()){

        // if(!gen3_robot.sendPosition(joint_traj.row(j).transpose()))
        // {
        //   std::cout<<"Send position error!!"<< std::endl;
        //   break;
        // }
        j++;
        gen3_robot.checkFeedback();
    }
    else
    {
      gen3_robot.getFeedback();
    }

    if(!(i%100))
    {
      auto message = sensor_msgs::msg::JointState();
      message.header.stamp = gen3_control_node->now();
      std::vector<double> position_vec;
      std::vector<double> velocity_vec;
      position_vec.resize(gen3_robot.q.size());
      velocity_vec.resize(gen3_robot.q_dot.size());
      Eigen::VectorXd::Map(&position_vec[0],position_vec.size()) = degreesToRadians(gen3_robot.q);
      Eigen::VectorXd::Map(&velocity_vec[0],velocity_vec.size()) = degreesToRadians(gen3_robot.q_dot);
      //Fill in message
      message.name = joint_names;
      message.position = position_vec;
      message.velocity = velocity_vec;
      //Publish
      joint_pub->publish(message);
    }
    
    loop_rate.sleep(); 
  }

  rclcpp::shutdown();
  return 0;
}