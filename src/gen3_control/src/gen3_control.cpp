// #include <robot_util.h>
#include <kortex_robot.h>
#include <rclcpp/rclcpp.hpp>
#include <math.h>
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
// Eigen::Matrix<double,4,7> cubicFunc;
Eigen::Matrix<double,6,7> quinticFunc;


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
  gen3_control_node->declare_parameter("enable_send_position",true);
  gen3_control_node->declare_parameter("pub_test_positions",false);
  gen3_control_node->declare_parameter("q_dot_alpha",0.1);
  gen3_control_node->declare_parameter("q_dotdot_alpha",0.05);
  int pos_pub_rate = gen3_control_node->get_parameter("pos_pub_rate").as_int();
  bool enable_send_position = gen3_control_node->get_parameter("enable_send_position").as_bool();
  bool pub_test_positions = gen3_control_node->get_parameter("pub_test_positions").as_bool();
  double q_dot_alpha = gen3_control_node->get_parameter("q_dot_alpha").as_double();
  double q_dotdot_alpha = gen3_control_node->get_parameter("q_dotdot_alpha").as_double();
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

  //Kortex_robot
  auto gen3_robot = kortex_robot(rate,q_dot_alpha,q_dotdot_alpha);

  gen3_robot.setLowLevelServoing();
  gen3_robot.getFeedback();
  gen3_robot.setBaseCommand();
  int pos_pub_period = hzToLoopNum(pos_pub_rate,rate);
  rclcpp::Rate loop_rate(rate);
  int i=0;

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
      // cubicFunc = findCubicFunction(gen3_robot.q,joint_pos_target,gen3_robot.q_dot,target_time);
      //TODO: Decide between raw or filtered vel
      quinticFunc = findQuinticFunction(gen3_robot.q,joint_pos_target,gen3_robot.q_dot_filtered,gen3_robot.q_dotdot,target_time);
    }
    //Get next joint position for timestep
    if(!traj_end_reached)
    {
      //Update joint values
      gen3_robot.checkFeedback();
      //Get desired Joint vales at time slice
      Eigen::Matrix<double,1,6> time_matrix;
      time_matrix.row(0) <<  1,time_slices(traj_position),pow(time_slices(traj_position),2),pow(time_slices(traj_position),3),pow(time_slices(traj_position),4),pow(time_slices(traj_position),5);
      next_joint_step = time_matrix*quinticFunc;
      if(enable_send_position)
      {
        //Send joint step
        if(!gen3_robot.sendPosition(next_joint_step.transpose()))
        {
          std::cout<<"Send position error!!"<< std::endl;
          break;
        }
      }
      if(pub_test_positions)
      {
        //Pub joint_pos
        auto test_message = std_msgs::msg::Float64MultiArray();
        test_message.data = eigenToStdVec(next_joint_step);
        joint_pos_test_pub->publish(test_message);
      }
      //Check if end of time slice
      traj_position++;
      if(traj_position == time_slices.size())
      {
        traj_end_reached = true;
      }
    }
    else
    {
      if(!gen3_robot.getFeedback())
      {
        std::cout<<"Get feedback error " << std::endl;
        break;
      }
    }

    if(!(i%pos_pub_period))
    {
      auto message = sensor_msgs::msg::JointState();
      //Fill in message
      message.header.stamp = gen3_control_node->now();
      message.name = joint_names;

      //TODO: Changed to send filtered velocity instead
      std::vector<double> out_pos = eigenToStdVec(degreesToRadians(gen3_robot.q));
      std::vector<double> out_vel = eigenToStdVec(degreesToRadians(gen3_robot.q_dot_filtered));
      std::vector<double> out_acc = eigenToStdVec(degreesToRadians(gen3_robot.q_dotdot));

      out_pos.resize(13);
      out_vel.resize(13);
      out_acc.resize(13);
      message.position = out_pos;
      message.velocity = out_vel;
      message.effort = out_acc;
      //Publish
      joint_pub->publish(message);

      //Pub joint_pos
        auto test_message = std_msgs::msg::Float64MultiArray();
        test_message.data = eigenToStdVec(degreesToRadians(gen3_robot.q_dot));
        joint_pos_test_pub->publish(test_message);
      } 
    loop_rate.sleep(); 
  }

  rclcpp::shutdown();
  return 0;
}