//Pinocchio
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include "pinocchio/algorithm/jacobian.hpp"
//Eigen
#include <Eigen/Dense>
//CPP libs
#include <cmath>

class robot_util
{
private:
    pinocchio::Model model;
    pinocchio::Data data;

    std::vector<pinocchio::JointIndex> list_of_joints_to_keep_unlocked_by_id = { 1, 2, 3, 4, 5, 6, 7 };
    std::vector<pinocchio::JointIndex> list_of_joints_to_lock_by_id;
    
    const std::string end_effector_name = "end_effector_link";
    //InvKin constants
    // const double eps  = 1e-4;
    const double eps  = 3e-3;
    const int IT_MAX  = 1000;
    // const double DT   = 1e-1;
    const double DT   = 10e-1;
    const double damp = 1e-6;
        
public:
    Eigen::VectorXd q_model;
    Eigen::VectorXd q_model_full;
    Eigen::VectorXd current_ee_position;
    Eigen::Quaterniond current_ee_orientation;
    Eigen::VectorXd q_desired;

    robot_util(std::string urdf_filename);
    ~robot_util();

    void updateEEPose(Eigen::VectorXd q);
    Eigen::VectorXd getFullQ(Eigen::VectorXd q);
    Eigen::VectorXd getQFromFullQ(Eigen::VectorXd full_q);
    Eigen::Matrix<double,4,7> findCubicFunction(Eigen::VectorXd startPos,Eigen::VectorXd endPos,Eigen::VectorXd startSpeed,double endTime);
    Eigen::MatrixXd createTrajectory(Eigen::VectorXd start_position,Eigen::VectorXd end_position,Eigen::VectorXd start_speed,double soft_joint_speed_limit);
    bool inverseKinematics(Eigen::Vector3d position,Eigen::Quaterniond orientation);
    Eigen::VectorXd radiansToDegrees(Eigen::VectorXd vec);
    Eigen::VectorXd degreesToRadians(Eigen::VectorXd vec);
};

//inverse kin current_ee_orientation

//conversions

//filter