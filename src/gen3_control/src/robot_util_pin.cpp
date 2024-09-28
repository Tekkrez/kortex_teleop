#include "robot_util_pin.h"

robot_util::robot_util(std::string urdf_filename){
    pinocchio::Model full_model;
    pinocchio::urdf::buildModel(urdf_filename, full_model);
    Eigen::VectorXd q = neutral(full_model);

    //Find joints that need to be fixed
    for (pinocchio::JointIndex joint_id = 1; joint_id < full_model.joints.size(); ++joint_id)
    {
        if (std::find(list_of_joints_to_keep_unlocked_by_id.begin(), list_of_joints_to_keep_unlocked_by_id.end(),
                    joint_id) != list_of_joints_to_keep_unlocked_by_id.end())
        continue;
        else
        {
        list_of_joints_to_lock_by_id.push_back(joint_id);
        }
    }

    model = buildReducedModel(full_model, list_of_joints_to_lock_by_id, q);
    data = pinocchio::Data(model);
}

robot_util::~robot_util(){}

// Convert normal q configuration to pinocchio configuration
//Updates q_model and q_model_full
Eigen::VectorXd robot_util::getFullQ(Eigen::VectorXd q)
{
  q_model = q;
  // Continuous joints are represented by 2 q's: cos(theta), sin(theta) in pinocchio
  Eigen::VectorXd full_q(model.nq);
  int element_count = 0;
  for (int i = 0; i < q.size(); i++)
  {
    if (model.joints[i + 1].nq() == 2)
    {
      full_q(element_count) = cos(q(i));
      full_q(element_count + 1) = sin(q(i));
      element_count += 2;
    }
    else
    {
      full_q(element_count) = q(i);
      element_count++;
    }
  }
  q_model_full = full_q;
  return full_q;
}

//Covert from pinocchio configuration to normal q configuration
//Does not update any class properties
//Does fmod on the values
Eigen::VectorXd robot_util::getQFromFullQ(Eigen::VectorXd full_q)
{
  Eigen::VectorXd q(model.nv);
  int element_count = 0;
  for(int i = 0; i<q.size(); i++)
  {
    if(model.joints[i+1].nq()==2)
    {
      q(i) = fmod(atan2(full_q(element_count+1),full_q(element_count)),2*M_PI);
      element_count+=2;
    }
    else
    {
      q(i) = fmod(full_q(element_count),2*M_PI);
      element_count+=1;
    }
  }
  return q;
}

//Updates the current_ee_position and current_ee_orientation
void robot_util::updateEEPose(Eigen::VectorXd q)
{
    q_model_full = getFullQ(q);
    pinocchio::forwardKinematics(model, data, q_model_full);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::GeometryData::SE3 ee_se3 = data.oMf[model.getFrameId(end_effector_name)];
    current_ee_position = ee_se3.translation();
    Eigen::Quaterniond quat_ee(ee_se3.rotation());
    current_ee_orientation = quat_ee;
}

//Finds coefficients for a cubic function. End speed is set to be 0
Eigen::Matrix<double,4,7> robot_util::findCubicFunction(Eigen::VectorXd startPos,Eigen::VectorXd endPos,Eigen::VectorXd startSpeed,double endTime){
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
Eigen::MatrixXd robot_util::createTrajectory(Eigen::VectorXd start_position,Eigen::VectorXd end_position,Eigen::VectorXd start_speed,double soft_joint_speed_limit)
{
  //Find Coefficients scaled by soft joint speed limit
  Eigen::VectorXd diff = end_position-start_position;
  double time_step = 0.001;
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

//Update current EE pose before calling this
//Stores result in q_desired
bool robot_util::inverseKinematics(Eigen::Vector3d position,Eigen::Quaterniond orientation){
  Eigen::VectorXd q = q_model_full;

  pinocchio::Data::Matrix6x J(6,model.nv);
  J.setZero();

  const pinocchio::SE3 oMdes(orientation.toRotationMatrix(),position);

  bool success = false;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d err;
  Eigen::VectorXd v(model.nv);

  for (int i = 0;; i++)
  {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::SE3 fMd = data.oMf[model.getFrameId(end_effector_name)].actInv(oMdes);
    err = pinocchio::log6(fMd).toVector(); // in joint frame
    if (err.norm() < eps)
    {
      success = true;
      break;
    }

    if (i >= IT_MAX)
    {
      success = false;
      break;
    }
    pinocchio::computeFrameJacobian(model, data, q, model.getFrameId(end_effector_name), J); // J in joint frame
    pinocchio::Data::Matrix6 Jlog;
    pinocchio::Jlog6(fMd.inverse(), Jlog);
    J = -Jlog * J;
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(model, q, v * DT);
    if (!(i % 2))
      std::cout << i << ": error = " << err.transpose() << std::endl;
  }

  if (success)
  {
    // std::cout << "Convergence achieved!" << std::endl;
    q_desired = q;
    return true;
  }
  else
  {
    // std::cout
    //   << "\nWarning: the iterative algorithm has not reached convergence to the desired precision"
    //   << std::endl;
    return false;
  }
}
//Does radians to degrees conversion and applies mod operation
Eigen::VectorXd robot_util::radiansToDegrees(Eigen::VectorXd vec)
{
  return vec*180/M_PI;
}

Eigen::VectorXd robot_util::degreesToRadians(Eigen::VectorXd vec)
{
  return vec*M_PI/180;
}