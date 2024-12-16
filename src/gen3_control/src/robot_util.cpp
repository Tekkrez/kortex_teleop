#include "robot_util.h"

//Finds coefficients for a cubic function. End speed is set to be 0
Eigen::Matrix<double,4,7> findCubicFunction(const Eigen::VectorXd& startPos,const Eigen::VectorXd& endPos,const Eigen::VectorXd& startSpeed,double endTime){
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

//Finds coefficients for a cubic function
Eigen::Matrix<double,4,7> findCubicFunction(const Eigen::VectorXd& startPos,const Eigen::VectorXd& endPos,const Eigen::VectorXd& startSpeed,const Eigen::VectorXd& endSpeed,double endTime){
  Eigen::Matrix<double,4,4> timeMatrix;
  timeMatrix(0,Eigen::all) <<  1,0,0,0;
  timeMatrix(1,Eigen::all) <<  0,1,0,0;
  timeMatrix(2,Eigen::all) <<  1,endTime,pow(endTime,2),pow(endTime,3);
  timeMatrix(3,Eigen::all) <<  0,1,2*endTime,3*pow(endTime,2);

  Eigen::Matrix<double,4,7> target;
  target.row(0) = startPos;
  target.row(1) = startSpeed;
  target.row(2) = endPos;
  target.row(3) = endSpeed;
  // std::cout<<"Target: " <<target<<std::endl;

  Eigen::Matrix<double,4,7> coeffs;
  coeffs = timeMatrix.colPivHouseholderQr().solve(target);
  // std::cout<<"Solution: " <<coeffs<<std::endl;

  return coeffs;
}

//Finds coefficients for a quintic function. End speed and acceleration is set to be 0
Eigen::Matrix<double,6,7> findQuinticFunction(const Eigen::VectorXd& startPos,const Eigen::VectorXd& endPos,const Eigen::VectorXd& startSpeed,const Eigen::VectorXd& startAccel,double endTime)
{
  Eigen::Matrix<double,6,6> timeMatrix;
  timeMatrix.row(0) <<  1,0,0,0,0,0;
  timeMatrix.row(1) <<  0,1,0,0,0,0;
  timeMatrix.row(2) <<  0,0,2,0,0,0;
  timeMatrix.row(3) <<  1,endTime,pow(endTime,2),pow(endTime,3),pow(endTime,4),pow(endTime,5);
  timeMatrix.row(4) <<  0,1,2*endTime,3*pow(endTime,2),4*pow(endTime,3),5*pow(endTime,4);
  timeMatrix.row(5) <<  0,0,2,6*endTime,12*pow(endTime,2),20*pow(endTime,3);

  Eigen::Matrix<double,6,7> target;
  target.row(0) = startPos;
  target.row(1) = startSpeed;
  target.row(2) = startAccel;
  target.row(3) = endPos;
  target.row(4) = Eigen::VectorXd::Zero(7);
  target.row(5) = Eigen::VectorXd::Zero(7);
  // std::cout<<"Target: " <<target<<std::endl;

  Eigen::Matrix<double,6,7> coeffs;
  coeffs = timeMatrix.colPivHouseholderQr().solve(target);
  // std::cout<<"Solution: " <<coeffs<<std::endl;
  return coeffs;
}
//Finds coefficients for a linear function with a defined constant slope
//All joints will get to target velocity at the same time
Eigen::Matrix<double,2,7> findLinearFunction(const Eigen::VectorXd& startPos, const Eigen::VectorXd& endPos, const double& endTime)
{
  Eigen::Matrix<double,2,2> timeMatrix;
  timeMatrix.row(0) << 1,0;
  timeMatrix.row(1) << 1,endTime;

  Eigen::Matrix<double,2,7> target;
  target.row(0) = startPos;
  target.row(1) = endPos;

  Eigen::Matrix<double,2,7> coeffs;
  coeffs = timeMatrix.colPivHouseholderQr().solve(target);
  return coeffs;
}


//Create cubic smooth trajectory using start and position for each of the joints
//Soft speed limit is used to determine time to complete trajectory
Eigen::MatrixXd createTrajectory(const Eigen::VectorXd& start_position,const Eigen::VectorXd& end_position,const Eigen::VectorXd& start_speed,double soft_joint_speed_limit,double time_step)
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

Eigen::VectorXd radiansToDegrees(const Eigen::VectorXd& vec)
{
  return vec*180/M_PI;
}

Eigen::VectorXd degreesToRadians(const Eigen::VectorXd& vec)
{
  return vec*M_PI/180;
}

std::vector<double> eigenToStdVec(const Eigen::VectorXd& vec)
{
  std::vector<double> vec_vec;
  vec_vec.resize(vec.size());
  Eigen::VectorXd::Map(&vec_vec[0],vec_vec.size()) = vec;
  return vec_vec;
}

Eigen::VectorXd stdVecToEigen(const std::vector<double>& vec)
{
  Eigen::VectorXd eig_vec(vec.size());
  std::copy(vec.begin(),vec.end(),eig_vec.data());
  return eig_vec;
}

//Given desired rate in hz, and frequency of main loop in hz
//Returns num of loops to achive desired rate
//Hz must be less than loopRate to achieve desired behaviour
int hzToLoopNum(const int& hz, const int& loopRate)
{
  return static_cast<int>(round(loopRate/hz));
}

//Initialize or reset filter
void LowPassFilter::lowPassFilterInit(const Eigen::VectorXd& initVector, const double& alpha_init)
{
  this->alpha = alpha_init;
  this->value_container = initVector;
};

Eigen::VectorXd LowPassFilter::applyFilter(const Eigen::VectorXd& new_vector)
{
  Eigen::VectorXd filteredVector(new_vector.size());
  filteredVector = new_vector * this->alpha + (1 - this->alpha) * this->value_container;
  this->value_container = filteredVector;
  return filteredVector;
}

Eigen::VectorXd LowPassFilter::applyFilterTimeScaled(const Eigen::VectorXd& new_vector,const double& time_delta,const double& min_alpha,const double& max_alpha_time)
{  
  Eigen::VectorXd filteredVector(new_vector.size());
  double t_alpha = (1-min_alpha)*std::tanh(2*time_delta/max_alpha_time)+min_alpha;
  filteredVector = new_vector * t_alpha + (1 - t_alpha) * this->value_container;
  this->value_container = filteredVector;
  return filteredVector;
}

