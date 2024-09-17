#include <robot_util.h>
#include <kortex_robot.h>
#include <rclcpp/rclcpp.hpp>

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
  robot_util gen3_util("/home/kevin/once_again/install/gen3_control/share/gen3_control/config/gen3_7dof_with_gripper.urdf");
  
  Eigen::VectorXd test_q(7);
  test_q << 0,0,0,0,0,0,0;
  gen3_util.updateEEPose(test_q);
  Eigen::Vector3d pos(0.3,0.3,0.3);
  Eigen::Quaterniond orien(1,0,0,0);
  try
  {
    
    RCLCPP_INFO(LOGGER,"IKin begin");
    if(gen3_util.inverseKinematics(pos,orien))
    {
      RCLCPP_INFO(LOGGER,"IKin Success");
      std::cout <<"Model.nv: " <<gen3_util.q_desired << std::endl;
      std::cout <<"Desired joint vals: " <<gen3_util.q_desired << std::endl;
      gen3_util.updateEEPose(gen3_util.q_desired);
      std::cout <<"Result Position: " << gen3_util.current_ee_position << std::endl;
      std::cout <<"Result Orientation: " << gen3_util.current_ee_orientation << std::endl;
    }
    else{
      RCLCPP_INFO(LOGGER,"IKin convergence Fail");
    }
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
  // while(rclcpp::ok())
  // {

  // }

  rclcpp::shutdown();
  return 0;
}