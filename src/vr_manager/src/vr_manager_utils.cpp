#include "vr_manager_utils.h"

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