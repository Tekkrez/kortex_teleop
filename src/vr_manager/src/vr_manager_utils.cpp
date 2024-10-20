#include "vr_manager_utils.h"
class PoseFilter
{
private:
  Eigen::VectorXd value_container;
  double alpha;

public:
  void PoseFilterInit(const Eigen::Isometry3d& initVector, const double& alpha_init)
  {
    this->alpha = alpha_init;
    this->value_container << initVector.translation(),initVector.rotation();
  };
  Eigen::VectorXd applyFilter(const Eigen::Isometry3d& new_vector)
  {
    Eigen::VectorXd filteredVector(7);
    filteredVector = new_vector * this->alpha + (1 - this->alpha) * this->value_container;
    this->value_container = filteredVector;

    return filteredVector;
  }
};