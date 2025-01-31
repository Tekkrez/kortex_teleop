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

  std::vector<Eigen::Isometry3d> find_pose_matching_ids(const std::vector<long>& common_ids, const std::vector<long>& current_ids, const ros2_aruco_interfaces::msg::ArucoMarkers& msg)
  {
      std::vector<Eigen::Isometry3d> matching_poses;

      for(auto id : common_ids)
      {
          auto it_head = std::find(current_ids.begin(),current_ids.end(),id);
          if(it_head != current_ids.end())
          {
              auto id_index = std::distance(current_ids.begin(),it_head);
              Eigen::Isometry3d temp_iso;
              tf2::fromMsg(msg.poses[id_index],temp_iso);
              matching_poses.emplace_back(temp_iso);
          }
          else
          {
              std::cout<<"ID ISN'T Shared"<<std::endl;
          }
      }
      return matching_poses;
  }

  // https://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
  Eigen::Quaterniond average_quaternion(const std::vector<Eigen::Quaterniond>& vec_quat)
  {
      Eigen::MatrixXd quat_matrix;
      quat_matrix.resize(4,vec_quat.size());
      for(int i = 0;i<vec_quat.size();i++)
      {
          quat_matrix.col(i) << vec_quat[i].x(),vec_quat[i].y(),vec_quat[i].z(),vec_quat[i].w();
      }
      Eigen::EigenSolver<Eigen::Matrix4d> solver(quat_matrix*quat_matrix.transpose());
      Eigen::VectorXd eigenvalues = solver.eigenvalues().real();
      Eigen::MatrixXd eigenvectors = solver.eigenvectors().real();

      double max_eigenvalue = eigenvalues(0);
      int max_eigenvalue_index = 0;
      for(int i=1;i<eigenvalues.size();i++)
      {
        if(eigenvalues(i)>max_eigenvalue)
        {
        max_eigenvalue = eigenvalues(i);
        max_eigenvalue_index = i;
        }
      }
      Eigen::Vector4d largest_eigenvector = eigenvectors.col(max_eigenvalue_index);

      Eigen::Quaterniond average_quaternion;
      average_quaternion.w() = largest_eigenvector(Eigen::last);
      average_quaternion.vec() = largest_eigenvector(Eigen::seq(0,2));
      average_quaternion.normalize();

      return average_quaternion;
  }
  // Average of available positions and "Average" of quaternion
  Eigen::Isometry3d find_best_pose(const std::vector<Eigen::Isometry3d> poses)
  {
      std::vector<Eigen::Quaterniond> vec_quat;
      Eigen::Vector3d running_sum;
      for(auto& pose : poses)
      {  
        vec_quat.emplace_back(Eigen::Quaterniond(pose.rotation()));   
        running_sum += pose.translation();
      }
      Eigen::Isometry3d best_pose;
      best_pose.translation() = running_sum/poses.size();
      best_pose.linear() = average_quaternion(vec_quat).toRotationMatrix();

      return best_pose;
  }