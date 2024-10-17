//Eigen
#include <Eigen/Dense>
//CPP libs
#include <cmath>
#include <vector>

Eigen::Matrix<double,4,7> findCubicFunction(const Eigen::VectorXd& startPos,const Eigen::VectorXd& endPos,const Eigen::VectorXd& startSpeed,double endTime);
Eigen::Matrix<double,4,7> findCubicFunction(const Eigen::VectorXd& startPos,const Eigen::VectorXd& endPos,const Eigen::VectorXd& startSpeed, const Eigen::VectorXd& endSpeed,double endTime);
Eigen::MatrixXd createTrajectory(const Eigen::VectorXd& start_position,const Eigen::VectorXd& end_position,const Eigen::VectorXd& start_speed,double soft_joint_speed_limit, double time_step = 0.001);
Eigen::VectorXd radiansToDegrees(const Eigen::VectorXd& vec);
Eigen::VectorXd degreesToRadians(const Eigen::VectorXd& vec);
std::vector<double> eigenToStdVec(const Eigen::VectorXd& vec);
Eigen::VectorXd stdVecToEigen(const std::vector<double>& vec);
//inverse kin current_ee_orientation

//conversions

//filter