#include "robot_util.h"

//Finds coefficients for a cubic function. End speed is set to be 0
Eigen::Matrix<double,4,7> findCubicFunction(Eigen::VectorXd startPos,Eigen::VectorXd endPos,Eigen::VectorXd startSpeed,double endTime){
  Eigen::Matrix<double,4,4> timeMatrix;
  timeMatrix(0,Eigen::all) <<  1,0,0,0;
  timeMatrix(1,Eigen::all) <<  0,1,0,0;
  timeMatrix(2,Eigen::all) <<  1,endTime,pow(endTime,2),pow(endTime,3);
  timeMatrix(3,Eigen::all) <<  0,1,2*endTime,3*pow(endTime,2);

  Eigen::Matrix<double,4,7> target;
  target.row(0) = startPos;
  target.row(1) = startSpeed;
  target.row(2) = endPos;
  target.row(3) = Eigen::VectorXd::Zero(7);
  // std::cout<<"Target: " <<target<<std::endl;

  Eigen::Matrix<double,4,7> coeffs;
  coeffs = timeMatrix.colPivHouseholderQr().solve(target);
  // std::cout<<"Solution: " <<coeffs<<std::endl;

  return coeffs;
}

//Create cubic smooth trajectory using start and position for each of the joints
//Soft speed limit is used to determine time to complete trajectory
Eigen::MatrixXd createTrajectory(Eigen::VectorXd start_position,Eigen::VectorXd end_position,Eigen::VectorXd start_speed,double soft_joint_speed_limit,double time_step)
{
  //Find Coefficients scaled by soft joint speed limit
  Eigen::VectorXd diff = end_position-start_position;
  double completion_time = diff.cwiseAbs().maxCoeff()/soft_joint_speed_limit;
  //Round to nearest time step
  completion_time = round(completion_time/time_step)*time_step;
  Eigen::Matrix<double,4,7> coeffs;
  coeffs = findCubicFunction(start_position,end_position,start_speed,completion_time);
  //Get time slices
  Eigen::VectorXd time_slices = Eigen::VectorXd::LinSpaced(completion_time/time_step,time_step,completion_time);
  //Create trajectory
  Eigen::MatrixXd joint_traj(time_slices.size(),7);
  //Fill in each waypoint in trajectory
  for (int i=0; i<time_slices.size();i++){
    Eigen::Matrix<double,1,4> time_matrix;
    time_matrix <<  1,time_slices(i),pow(time_slices(i),2),pow(time_slices(i),3);
    // time_matrix.row(1) <<  0,1,2*time_slices(i),3*pow(time_slices(i),2);
    Eigen::Matrix<double,1,7> result;
    result = time_matrix*coeffs;
    joint_traj.row(i) = result;
    }
  // RCLCPP_INFO(LOGGER,"Here 2");
  return joint_traj;
}

Eigen::VectorXd radiansToDegrees(Eigen::VectorXd vec)
{
  return vec*180/M_PI;
}

Eigen::VectorXd degreesToRadians(Eigen::VectorXd vec)
{
  return vec*M_PI/180;
}

//Does radians to degrees conversion and applies mod operation