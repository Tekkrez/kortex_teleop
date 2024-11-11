//Eigen
#include <Eigen/Dense>
//CPP libs
#include <iostream>
#include <cmath>

Eigen::Matrix<double,4,7> findCubicFunction(const Eigen::VectorXd& startPos,const Eigen::VectorXd& endPos,const Eigen::VectorXd& startSpeed,double endTime);
Eigen::Matrix<double,6,7> findQuinticFunction(const Eigen::VectorXd& startPos,const Eigen::VectorXd& endPos,const Eigen::VectorXd& startSpeed,const Eigen::VectorXd& startAccel,double endTime);
Eigen::MatrixXd createTrajectory(const Eigen::VectorXd& start_position,const Eigen::VectorXd& end_position,const Eigen::VectorXd& start_speed,double soft_joint_speed_limit,double time_step = 0.001);
Eigen::VectorXd radiansToDegrees(const Eigen::VectorXd& vec);
Eigen::VectorXd degreesToRadians(const Eigen::VectorXd& vec);
Eigen::VectorXd vectorFMod(Eigen::VectorXd vec);

//inverse kin current_ee_orientation

//conversions

//filter