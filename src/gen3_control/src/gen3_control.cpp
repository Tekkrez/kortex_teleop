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

  rclcpp::Rate loop_rate(rate);
  // auto time_point1 = chrono::high_resolution_clock::now();
  // auto time_point2 = chrono::high_resolution_clock::now();
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
    //Check for new message
    if(new_trajectory)
    {
    //   RCLCPP_WARN_STREAM(LOGGER, "Here 1"<<"\n");
      new_trajectory=false;
      waypoint_position = 0;
      waypoint_reached = true;
    }
    //Find new cubic function to waypoint
    if(trajectory_ready && waypoint_reached)
    {
      //check if end of waypoints is reached
      if(waypoint_position == waypoint_end)
      {
        trajectory_ready = false;
      }
      else//get function to waypoint
      {
        Eigen::VectorXd target_pos(7);
        Eigen::VectorXd target_vel(7);
        std::copy(trajectory.points[waypoint_position].positions.begin(),trajectory.points[waypoint_position].positions.end(),target_pos.data());
        std::copy(trajectory.points[waypoint_position].velocities.begin(),trajectory.points[waypoint_position].velocities.end(),target_vel.data());
        Eigen::VectorXd diff = gen3_robot.q - radiansToDegrees(target_pos);
        if(diff.cwiseAbs().maxCoeff()>180){
          for (int i : gen3_robot.continuous_joints)
          {
            if(abs(diff(i))>180)
            {
              RCLCPP_WARN_STREAM(LOGGER, "Large desired diff in continuous joint. Look at Joint: "<<i+1<<"\n");
            }
          }
        }
        cubicFunc = findCubicFunction(gen3_robot.q,radiansToDegrees(target_pos),gen3_robot.q_dot,radiansToDegrees(target_vel),waypoint_interval);
        waypoint_reached = false;
        time_slice_position = 0;
        waypoint_position++;
        std::cout << "Waypoint position: " << waypoint_position << "/" << waypoint_end << std::endl;
        std::cout << "Current pos: " << gen3_robot.q.transpose() << "\nCurrent target: " << radiansToDegrees(target_pos).transpose()<<std::endl;
        std::cout << "Current vel: " << gen3_robot.q_dot.transpose() << "\nCurrent target: " << radiansToDegrees(target_vel).transpose()<<std::endl;
      }
    }
    //Get next joint position for timestep
    if(trajectory_ready && !waypoint_reached)
    {
      Eigen::Matrix<double,1,4> time_matrix;
      time_matrix <<  1,time_slices(time_slice_position),pow(time_slices(time_slice_position),2),pow(time_slices(time_slice_position),3);
      //Desired Joint vales at time slice
      next_joint_step = time_matrix*cubicFunc;
      //Send joint step
      // if(!gen3_robot.sendPosition(next_joint_step.transpose()))
      //   {
      //     std::cout<<"Send position error!!"<< std::endl;
      //     break;
      //   }
      //Check if end of time slice
      time_slice_position++;
      if(time_slice_position == time_slice_end)
      {
        waypoint_reached = true;
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

    if(!(i%100))
    {
      gen3_robot.checkFeedback();
      // gen3_util.updateEEPose(gen3_robot.q);
      // // std::cout<<"Step: "<< i <<" - sleep time: "<< elapsed.count() <<" - run time: "<< runTime.count()  <<" - Joint Positions: " << gen3_robot.q.transpose() << std::endl;
      // std::cout<<"Desired Joint position: " << next_joint_step << std::endl;
      // std::cout <<"Current joint position: " << gen3_robot.q << std::endl;
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