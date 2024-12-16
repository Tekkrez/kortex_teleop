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
const int rate = 1100;
const double period=1/static_cast<double>(rate);
//Variables for trajectory tracking
Eigen::VectorXd joint_vel_target(7);
bool new_velocity = false;
bool vel_target_reached = true;
int traj_position = 0;
//Soft accel limit
//TODO: Setup as param
double soft_accel_limit = 720;
Eigen::Matrix<double,1,7> next_joint_vel;
Eigen::VectorXd time_slices;
Eigen::Matrix<double,2,7> linear_func;
Eigen::VectorXd vel_command(7);

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
  rclcpp::Rate loop_rate(rate);
  int i=0;
  int pos_pub_period = hzToLoopNum(pos_pub_rate,rate);
  //Need this since if using live joint position to rotate the joint, 
  //the resulting velocities are much lower than expected and the joints drift toward the ground
  Eigen::VectorXd tracked_pos = gen3_robot.q;

  // auto timePoint = std::chrono::high_resolution_clock::now();
  while(rclcpp::ok())
  {
    i++;

    gen3_robot.checkFeedback();
    // Check for new message
    if(new_velocity)
    {
      new_velocity = false;
      vel_target_reached = false;
      traj_position = 0;
      double completion_time = (joint_vel_target-gen3_robot.q_dot).cwiseAbs().maxCoeff()/soft_accel_limit;
      
      
      if(completion_time < 2*period)
      {
        vel_target_reached = true;
        vel_command = joint_vel_target;
      }
      else
      {
        time_slices = Eigen::VectorXd::LinSpaced(completion_time*rate, period, completion_time);
        linear_func = findLinearFunction(gen3_robot.q_dot_filtered,joint_vel_target,completion_time);

        // std::cout << "Completion_time: " << completion_time << std::endl; 
        // std::cout << "Current vel: " << gen3_robot.q_dot_filtered.transpose() << std::endl; 
        // std::cout << "Target vel: " << joint_vel_target.transpose() << std::endl; 
        // std::cout << "linear_function: " << linear_func << std::endl; 
      }
    }
    //Get next joint position for timestep
    if(!vel_target_reached)
    {
      //Get velocity
      Eigen::Matrix<double,1,2> time_matrix;
      time_matrix.row(0) << 1,time_slices(traj_position);

      Eigen::Matrix<double,1,7> result;
      result = time_matrix*linear_func;
      vel_command = result;
      traj_position++;
    }

    if(enable_send_position)
    {
      //FIXME: 
      tracked_pos += vel_command*period;
      if(!gen3_robot.sendPosition(tracked_pos))
      {
        std::cout<<"Send velocity error!!"<< std::endl;
        break;
      }
    }
    if(pub_test_positions)
    {
      //Pub joint_vel
      auto test_message = std_msgs::msg::Float64MultiArray();
      test_message.data = eigenToStdVec(vel_command);
      joint_pos_test_pub->publish(test_message);
    }
    //Check if end of time slice
    if(traj_position == time_slices.size())
    {
      vel_target_reached = true;
    }
    // else
    // {
    //   if(!gen3_robot.getFeedback())
    //   {
    //     std::cout<<"Get feedback error " << std::endl;
    //     break;
    //   }
    // }

    if(!(i%pos_pub_period))
    {
      auto message = sensor_msgs::msg::JointState();
      //Fill in message
      message.header.stamp = gen3_control_node->now();
      message.name = joint_names;

      std::vector<double> out_pos = eigenToStdVec(degreesToRadians(gen3_robot.q));
      std::vector<double> out_vel = eigenToStdVec(degreesToRadians(gen3_robot.q_dot_filtered));
      std::vector<double> out_acc = eigenToStdVec(degreesToRadians(gen3_robot.q_dotdot));
      //Add zeros at the end for the gripper joints states
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
        test_message.data = eigenToStdVec(degreesToRadians(vel_command));
        joint_pos_test_pub->publish(test_message);
      }
    // auto duration = std::chrono::duration_cast<microseconds>(std::chrono::high_resolution_clock::now()-timePoint);
    // std::cout<<"Loop Time: "<< duration.count() << std::endl;
    // timePoint = std::chrono::high_resolution_clock::now();
    loop_rate.sleep(); 
  }

  rclcpp::shutdown();
  return 0;
}