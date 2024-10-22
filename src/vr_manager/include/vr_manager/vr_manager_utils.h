#include "Eigen/Dense"

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