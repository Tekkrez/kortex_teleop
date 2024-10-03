//ROS stuff
#include <rclcpp/rclcpp.hpp>

//Eigen
#include <Eigen/Dense>

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

    k_api::Base::ServoingModeInformation servoing_mode;
    bool low_level_servoing = false;
public:
    //Degrees
    Eigen::VectorXd q;
    //Degrees
    Eigen::VectorXd q_dot;
    kortex_robot();
    ~kortex_robot();
    
    void kExceptionHandle(k_api::KDetailedException& ex);
    void stdExceptionHandle(std::runtime_error& ex);

    void setLowLevelServoing();
    void setSingleLevelServoing();
    bool getFeedback();
    void checkFeedback();

    bool sendPosition(const Eigen::VectorXd& desired_q_step);
    bool setBaseCommand();
};