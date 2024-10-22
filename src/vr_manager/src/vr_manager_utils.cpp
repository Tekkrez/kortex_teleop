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

  Eigen::VectorXd LowPassFilter::applyFilterTimeScaled(const Eigen::VectorXd& new_vector,const double& time_delta,const double& min_alpha,const double& max_alpha_time)
  {
    Eigen::VectorXd filteredVector(new_vector.size());
    double t_alpha = (1-min_alpha)*std::tanh(2*time_delta/max_alpha_time)+min_alpha;
    filteredVector = new_vector * t_alpha + (1 - t_alpha) * this->value_container;
    this->value_container = filteredVector;
    return filteredVector;
  }