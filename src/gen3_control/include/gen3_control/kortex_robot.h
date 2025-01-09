//ROS stuff
#include <rclcpp/rclcpp.hpp>

//Eigen
#include <eigen3/Eigen/Dense>

//Kortex stuff
#include "KDetailedException.h"
// #include <KDetailedException.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include <google/protobuf/util/json_util.h>

#include <unistd.h>
#include <time.h>
#include <cmath>
// Custom include
#include <chrono>
#include <robot_util.h>



namespace k_api = Kinova::Api;

class kortex_robot
{
private:
    k_api::TransportClientTcp* tcp_transport;
    k_api::RouterClient* tcp_router;
    k_api::SessionManager* tcp_session_manager;
    k_api::TransportClientUdp* udp_transport;
    k_api::RouterClient* udp_router;
    k_api::SessionManager* udp_session_manager;

    k_api::Base::BaseClient* base;
    k_api::BaseCyclic::BaseCyclicClient* base_cyclic;
    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command  base_command;
    k_api::GripperCyclic::MotorCommand* gripper_command;

    k_api::Base::ServoingModeInformation servoing_mode;
    bool low_level_servoing = false;
public:
    //Degrees
    Eigen::VectorXd q;
    //Degrees/second
    Eigen::VectorXd q_dot;
    Eigen::VectorXd q_dot_filtered;
    //Degrees/second^2
    Eigen::VectorXd q_dotdot;
    Eigen::VectorXd prev_q_dot;
    double period;
    double minimum_position_error = 1.5;
    double gripper_position;
    LowPassFilter vel_filt;
    LowPassFilter accel_filt;

    std::vector<int> continuous_joints = {0,2,4,6};
    std::vector<int> non_continuous_joints = {1,3,5};
    std::vector<double> joint_limits = {0,138,0,152,0,127,0};
    kortex_robot(int rate,double q_dot_alpha,double q_dotdot_alpha);
    ~kortex_robot();
    
    void kExceptionHandle(k_api::KDetailedException& ex);
    void stdExceptionHandle(std::runtime_error& ex);

    void setLowLevelServoing();
    void setSingleLevelServoing();
    bool getFeedback();
    void checkFeedback();

    bool sendPosition(const Eigen::VectorXd& desired_q_step);
    bool sendGripperPosition(double target_position,double proportional_gain=2.0);
    bool updateGripperPosition(double target_position,double proportional_gain=2.0);
    // bool sendVelocity(const Eigen::VectorXd& desired_vel);
    bool setBaseCommand();
};