//Eigen
#include <Eigen/Dense>
//CPP libs
#include <cmath>

Eigen::Matrix<double,4,7> findCubicFunction(Eigen::VectorXd startPos,Eigen::VectorXd endPos,Eigen::VectorXd startSpeed,double endTime);
Eigen::Matrix<double,4,7> findCubicFunction(Eigen::VectorXd startPos,Eigen::VectorXd endPos,Eigen::VectorXd startSpeed, Eigen::VectorXd endSpeed,double endTime);
Eigen::MatrixXd createTrajectory(Eigen::VectorXd start_position,Eigen::VectorXd end_position,Eigen::VectorXd start_speed,double soft_joint_speed_limit, double time_step = 0.001);
Eigen::VectorXd radiansToDegrees(Eigen::VectorXd vec);
Eigen::VectorXd degreesToRadians(Eigen::VectorXd vec);

//inverse kin current_ee_orientation

//conversions

//filter