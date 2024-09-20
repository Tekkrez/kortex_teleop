#include <robot_util.h>
#include <kortex_robot.h>
#include <rclcpp/rclcpp.hpp>
#include <math.h>
#include <chrono>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_control_node");

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

  //Robot stuff
  robot_util gen3_util("/home/kevin/kortex_teleop/install/gen3_control/share/gen3_control/config/gen3_7dof_with_gripper.urdf");
  
  

  auto gen3_robot = kortex_robot();
  gen3_robot.setLowLevelServoing();
  gen3_robot.getFeedback();
  gen3_robot.setBaseCommand();

  gen3_util.updateEEPose(gen3_robot.q);
  std::cout<<"EE start position: " << gen3_util.current_ee_position.transpose() << std::endl;
  std::cout<<"EE start orientation: " << gen3_util.current_ee_orientation << std::endl;

  Eigen::Vector3d posChange;
  posChange << 0.3,0.3,0.5;
  // Eigen::Quaterniond orien(1,0,0,0);
  try
  {
    RCLCPP_INFO(LOGGER,"IKin begin");
    if(gen3_util.inverseKinematics(posChange,gen3_util.current_ee_orientation))
    {
      std::cout <<"Standard q vals: " <<gen3_util.getQFromFullQ(gen3_util.q_desired).transpose() << std::endl;
    }
    else{
      RCLCPP_INFO(LOGGER,"IKin convergence Fail");
    }
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }


  std::cout << "Start Pos" << gen3_robot.q.transpose() <<std::endl;
  std::cout << "Start vel" << gen3_robot.q_dot.transpose() <<std::endl;

  Eigen::VectorXd endPos = gen3_util.radiansToDegrees(gen3_util.getQFromFullQ(gen3_util.q_desired));
  std::cout << "End Joint position" << endPos.transpose() <<std::endl;
  Eigen::MatrixXd joint_pos_traj = gen3_util.createTrajectory(gen3_robot.q,endPos,gen3_robot.q_dot,20);
  gen3_util.updateEEPose(gen3_util.degreesToRadians(joint_pos_traj.row(joint_pos_traj.rows()-1)));
  std::cout<<"EE end position" << gen3_util.current_ee_position << std::endl;
  
  std::cout<<"joint traj end position" << gen3_util.current_ee_position << std::endl;


  gen3_robot.setLowLevelServoing();
  
  rclcpp::Rate loop_rate(1050);
  auto prev_time = chrono::high_resolution_clock::now();
  auto time_point = chrono::high_resolution_clock::now();
  for(int i=1; i<joint_pos_traj.rows();i++)
  {
    time_point = chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(time_point - prev_time);
    // if(elapsed.count()<200)
    // {
    //   std::cout << "Concerning Elapsed time: " << elapsed.count() << std::endl;
    // }

    if(!gen3_robot.sendPosition(joint_pos_traj.row(i).transpose()))
    {
      std::cout<<"Send position error!!"<< std::endl;
      break;
    }

    prev_time = chrono::high_resolution_clock::now();

    // gen3_robot.getFeedback();
    if(!(i%100))
    {
      auto runTime = std::chrono::duration_cast<std::chrono::microseconds>(prev_time - time_point);

      gen3_robot.checkFeedback();
      gen3_util.updateEEPose(gen3_robot.q);
      std::cout<<"Step: "<< i <<" - sleep time: "<< elapsed.count() <<" - run time: "<< runTime.count()  <<" - Joint Positions: " << gen3_robot.q.transpose() << std::endl;
      std::cout<<"Desired Joint position: " << joint_pos_traj.row(i) << std::endl;

      std::cout<<"EE position: " << gen3_util.current_ee_position.transpose()<<"\n" << std::endl;
      
      // std::cout<<"EE start orientation: " << gen3_util.current_ee_orientation << std::endl;
      // std::cout << "Elapsed time: " << elapsed.count() << std::endl;
    }

    loop_rate.sleep();
  }

  // while(rclcpp::ok())
  // {
    
  //   //spin joint 7
  //   gen3_robot.sendPosition(test_q_pos);
  //   loop_rate.sleep(); 
  // }

  rclcpp::shutdown();
  return 0;
}