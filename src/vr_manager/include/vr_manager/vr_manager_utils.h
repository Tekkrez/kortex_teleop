#include "Eigen/Dense"
#include "vector"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

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

std::vector<Eigen::Isometry3d> find_pose_matching_ids(const std::vector<long>& common_ids, const std::vector<long>& current_ids, const ros2_aruco_interfaces::msg::ArucoMarkers& msg);
Eigen::Quaterniond average_quaternion(const std::vector<Eigen::Quaterniond>& vec_quat);
Eigen::Isometry3d find_best_pose(const std::vector<Eigen::Isometry3d> poses);