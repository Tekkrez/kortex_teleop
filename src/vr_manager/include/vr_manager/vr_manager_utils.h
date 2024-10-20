#include "Eigen/Dense"

class PoseFilter
{
private:
  Eigen::VectorXd value_container;
  double alpha;

public:
  void PoseFilterInit(const Eigen::Isometry3d& initVector, const double& alpha_init);
  Eigen::VectorXd applyFilter(const Eigen::Isometry3d& new_vector);
};