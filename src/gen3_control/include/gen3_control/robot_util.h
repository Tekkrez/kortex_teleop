//CPP libs
#include <cmath>
#include <vector>
#include <iostream>
//Eigen
#include <eigen3/Eigen/Dense>

Eigen::Matrix<double,4,7> findCubicFunction(const Eigen::VectorXd& startPos,const Eigen::VectorXd& endPos,const Eigen::VectorXd& startSpeed,double endTime);
Eigen::Matrix<double,4,7> findCubicFunction(const Eigen::VectorXd& startPos,const Eigen::VectorXd& endPos,const Eigen::VectorXd& startSpeed, const Eigen::VectorXd& endSpeed,double endTime);
Eigen::Matrix<double,6,7> findQuinticFunction(const Eigen::VectorXd& startPos,const Eigen::VectorXd& endPos,const Eigen::VectorXd& startSpeed,const Eigen::VectorXd& startAccel,double endTime);
Eigen::MatrixXd createTrajectory(const Eigen::VectorXd& start_position,const Eigen::VectorXd& end_position,const Eigen::VectorXd& start_speed,double soft_joint_speed_limit, double time_step = 0.001);
Eigen::VectorXd radiansToDegrees(const Eigen::VectorXd& vec);
Eigen::VectorXd degreesToRadians(const Eigen::VectorXd& vec);
std::vector<double> eigenToStdVec(const Eigen::VectorXd& vec);
Eigen::VectorXd stdVecToEigen(const std::vector<double>& vec);
int hzToLoopNum(const int& hz, const int& loopRate);


class LowPassFilter
{
  private:
    Eigen::VectorXd value_container;
    double alpha;

  public:
    void lowPassFilterInit(const Eigen::VectorXd& initVector, const double& alpha_init);
    Eigen::VectorXd applyFilter(const Eigen::VectorXd& new_vector);
    Eigen::VectorXd applyFilterTimeScaled(const Eigen::VectorXd& new_vector,const double& time_delta,const double& min_alpha=0.1,const double& max_alpha_time=0.2);

};