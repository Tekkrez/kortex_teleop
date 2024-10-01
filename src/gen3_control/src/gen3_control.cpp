#include <robot_util.h>
#include <kortex_robot.h>
#include <rclcpp/rclcpp.hpp>
#include <math.h>
#include <chrono>
#include <sensor_msgs/msg/joint_state.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_control_node");
std::vector<std::string> joint_names = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7" };

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

  //Robot stuff
  // robot_util gen3_util("/home/kevin/kortex_teleop/install/gen3_control/share/gen3_control/config/gen3_7dof_with_gripper.urdf");
  
  auto gen3_robot = kortex_robot();
  gen3_robot.setLowLevelServoing();
  gen3_robot.getFeedback();
  gen3_robot.setBaseCommand();

  // gen3_util.updateEEPose(gen3_robot.q);
  // std::cout<<"EE start position: " << gen3_util.current_ee_position.transpose() << std::endl;
  // std::cout<<"EE start orientation: " << gen3_util.current_ee_orientation << std::endl;

  // Eigen::Vector3d posChange;
  // posChange << 0.3,0.3,0.5;
  // // Eigen::Quaterniond orien(1,0,0,0);
  // try
  // {
  //   RCLCPP_INFO(LOGGER,"IKin begin");
  //   if(gen3_util.inverseKinematics(posChange,gen3_util.current_ee_orientation))
  //   {
  //     std::cout <<"Standard q vals: " <<gen3_util.getQFromFullQ(gen3_util.q_desired).transpose() << std::endl;
  //   }
  //   else{
  //     RCLCPP_INFO(LOGGER,"IKin convergence Fail");
  //   }
  // }
  // catch(const std::exception& e)
  // {
  //   std::cerr << e.what() << '\n';
  // }


  // std::cout << "Start Pos" << gen3_robot.q.transpose() <<std::endl;
  // std::cout << "Start vel" << gen3_robot.q_dot.transpose() <<std::endl;

  // Eigen::VectorXd endPos = gen3_util.radiansToDegrees(gen3_util.getQFromFullQ(gen3_util.q_desired));
  // std::cout << "End Joint position" << endPos.transpose() <<std::endl;
  // Eigen::MatrixXd joint_pos_traj = gen3_util.createTrajectory(gen3_robot.q,endPos,gen3_robot.q_dot,15);
  // gen3_util.updateEEPose(gen3_util.degreesToRadians(joint_pos_traj.row(joint_pos_traj.rows()-1)));
  // std::cout<<"EE end position" << gen3_util.current_ee_position << std::endl;
  
  // std::cout<<"joint traj end position" << gen3_util.current_ee_position << std::endl;


  // gen3_robot.setLowLevelServoing();

  rclcpp::Rate loop_rate(1100);
  auto time_point1 = chrono::high_resolution_clock::now();
  auto time_point2 = chrono::high_resolution_clock::now();
  // for(int i=1; i<joint_pos_traj.rows();i++)
  // {
  //   time_point = chrono::high_resolution_clock::now();
  //   auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(time_point - prev_time);
  //   // if(elapsed.count()<200)
  //   // {
  //   //   std::cout << "Concerning Elapsed time: " << elapsed.count() << std::endl;
  //   // }

  //   if(!gen3_robot.sendPosition(joint_pos_traj.row(i).transpose()))
  //   {
  //     std::cout<<"Send position error!!"<< std::endl;
  //     break;
  //   }

  //   prev_time = chrono::high_resolution_clock::now();

  //   // gen3_robot.getFeedback();
    // if(!(i%100))
    // {
    //   auto runTime = std::chrono::duration_cast<std::chrono::microseconds>(prev_time - time_point);

    //   // gen3_robot.checkFeedback();
    //   // gen3_util.updateEEPose(gen3_robot.q);
    //   // // std::cout<<"Step: "<< i <<" - sleep time: "<< elapsed.count() <<" - run time: "<< runTime.count()  <<" - Joint Positions: " << gen3_robot.q.transpose() << std::endl;
    //   // // std::cout<<"Desired Joint position: " << joint_pos_traj.row(i) << std::endl;

    //   // // std::cout<<"EE position: " << gen3_util.current_ee_position.transpose()<<"\n" << std::endl;
      
    //   auto message = sensor_msgs::msg::JointState();
    //   std::vector<double> position_vec;
    //   std::vector<double> velocity_vec;
    //   position_vec.resize(gen3_robot.q.size());
    //   velocity_vec.resize(gen3_robot.q_dot.size());
    //   Eigen::VectorXd::Map(&position_vec[0],position_vec.size()) = gen3_robot.q;
    //   Eigen::VectorXd::Map(&velocity_vec[0],velocity_vec.size()) = gen3_robot.q_dot;
    //   //Fill in message
    //   message.name = joint_names;
    //   message.position = position_vec;
    //   message.velocity = velocity_vec;
    //   //Publish
    //   joint_pub->publish(message);

      // std::cout<<"EE start orientation: " << gen3_util.current_ee_orientation << std::endl;
      // std::cout << "Elapsed time: " << elapsed.count() << std::endl;
  // }

  //   loop_rate.sleep();
  // }
  int i=0;
  while(rclcpp::ok())
  {
    i++;
    if(!gen3_robot.getFeedback())
    {
      std::cout<<"Get feedback error " << std::endl;
      break;
    }

    if(!(i%100))
    {
      // gen3_robot.checkFeedback();
      // gen3_util.updateEEPose(gen3_robot.q);
      // // std::cout<<"Step: "<< i <<" - sleep time: "<< elapsed.count() <<" - run time: "<< runTime.count()  <<" - Joint Positions: " << gen3_robot.q.transpose() << std::endl;
      // // std::cout<<"Desired Joint position: " << joint_pos_traj.row(i) << std::endl;

      // // std::cout<<"EE position: " << gen3_util.current_ee_position.transpose()<<"\n" << std::endl;
      // auto time_point1 = chrono::high_resolution_clock::now();

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
      // auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(chrono::high_resolution_clock::now()-time_point1);
      // std::cout << "Current q: " << degreesToRadians(gen3_robot.q).transpose() << std::endl;
    }
    
    loop_rate.sleep(); 
  }

  rclcpp::shutdown();
  return 0;
}