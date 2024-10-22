#include "Eigen/Dense"

class LowPassFilter
{
  private:
    Eigen::VectorXd value_container;
    double alpha;

  public:
    void lowPassFilterInit(const Eigen::VectorXd& initVector, const double& alpha_init);
    Eigen::VectorXd applyFilter(const Eigen::VectorXd& new_vector);
};