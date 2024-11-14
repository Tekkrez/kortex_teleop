#include "kortex_robot.h"

void kortex_robot::kExceptionHandle(k_api::KDetailedException& ex)
{
    // You can print the error informations and error codes
    auto error_info = ex.getErrorInfo().getError();
    std::cout << "KDetailedoption detected what:  " << ex.what() << std::endl;
    
    std::cout << "KError error_code: " << error_info.error_code() << std::endl;
    std::cout << "KError sub_code: " << error_info.error_sub_code() << std::endl;
    std::cout << "KError sub_string: " << error_info.error_sub_string() << std::endl;

    // Error codes by themselves are not very verbose if you don't see their corresponding enum value
    // You can use google::protobuf helpers to get the string enum element for every error code and sub-code 
    std::cout << "Error code string equivalent: " << k_api::ErrorCodes_Name(k_api::ErrorCodes(error_info.error_code())) << std::endl;
    std::cout << "Error sub-code string equivalent: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(error_info.error_sub_code())) << std::endl;
}

void kortex_robot::stdExceptionHandle(std::runtime_error& ex)
{
    std::cout << "Runtime error: " << ex.what() << std::endl;
}

void kortex_robot::setLowLevelServoing(){
    // Set the base in low-level servoing mode
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);
    low_level_servoing = true;
}

void kortex_robot::setSingleLevelServoing(){
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);
    low_level_servoing = false;
}

kortex_robot::kortex_robot(int rate,double q_dot_alpha,double q_dotdot_alpha){
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };

    tcp_transport = new k_api::TransportClientTcp();
    tcp_router = new k_api::RouterClient(tcp_transport,error_callback);
    tcp_transport->connect("192.168.1.10",10000);

    udp_transport = new k_api::TransportClientUdp();
    udp_router = new k_api::RouterClient(udp_transport,error_callback);
    udp_transport->connect("192.168.1.10",10001);

    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    base = new k_api::Base::BaseClient(tcp_router);
    base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(udp_router);
    
    servoing_mode = k_api::Base::ServoingModeInformation();
    try
    {
        tcp_session_manager = new k_api::SessionManager(tcp_router);
        tcp_session_manager->CreateSession(create_session_info);
        udp_session_manager = new k_api::SessionManager(udp_router);
        udp_session_manager->CreateSession(create_session_info);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    try
    {
        base->ClearFaults();
    }
    catch(...)
    {
        std::cout << "Unable to clear faults" << '\n';
    }
    q.resize(7);
    q_dot.resize(7);
    q_dotdot.resize(7);
    q_dot_filtered.resize(7);
    prev_q_dot=q_dot;
    period = 1/static_cast<double>(rate);
    accel_filt.lowPassFilterInit(Eigen::VectorXd(7), q_dotdot_alpha);    
    vel_filt.lowPassFilterInit(Eigen::VectorXd(7), q_dot_alpha);
}

kortex_robot::~kortex_robot(){
    try
    {
        base->ApplyEmergencyStop(0, {false, 0, 100});
        // clear faults
        base->ClearFaults();
    }
    catch (k_api::KDetailedException& ex)
    {
        kExceptionHandle(ex);
    }

    setLowLevelServoing();
    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    //close API and disconnect
    tcp_session_manager->CloseSession();
    udp_session_manager->CloseSession();
    tcp_router->SetActivationStatus(false);
    tcp_transport->disconnect();
    udp_router->SetActivationStatus(false);
    udp_transport->disconnect();

    delete base;
    delete base_cyclic;
    delete tcp_session_manager;
    delete udp_session_manager;
    delete tcp_router;
    delete udp_router;
    delete tcp_transport;
    delete udp_transport;
}
//Updates q and q dot
//Needs to be in low level servoing mode
bool kortex_robot::getFeedback()
{
    if(!low_level_servoing){
        return false;
    }

    base_feedback = base_cyclic->RefreshFeedback();
    for(int i=0; i<q.size();i++)
    {
        q(i) = base_feedback.actuators(i).position();
        //Make it in the range of [-x,x] instead of the kortex default of {[0,x],[360-x,360)} for non_continuous_joints
        //Do it to all of them for consistency
        if(q(i)>180)
        {
            q(i) = q(i)-360;
        }
        q_dot(i) = base_feedback.actuators(i).velocity();
    }
    q_dot_filtered = vel_filt.applyFilter(q_dot);
    q_dotdot = accel_filt.applyFilter((q_dot_filtered-prev_q_dot)/(period));
    prev_q_dot = q_dot_filtered;
    return true;    
}

//Checks feedback if it is being updated by other functions like sendPosition
void kortex_robot::checkFeedback()
{
     for(int i=0; i<q.size();i++)
    {
        q(i) = base_feedback.actuators(i).position();
        //Make it in the range of [-x,x] instead of the kortex default of {[0,x],[360-x,360)} for non_continuous_joints
        //Do it to all of them for consistency
        if(q(i)>180)
        {
            q(i) = q(i)-360;
        }
        q_dot(i) = base_feedback.actuators(i).velocity();
    }
    q_dot_filtered = vel_filt.applyFilter(q_dot);
    q_dotdot = accel_filt.applyFilter((q_dot_filtered-prev_q_dot)/(period));
    prev_q_dot=q_dot_filtered;
}
//Initializes base_command. Calls getFeedback() first
bool kortex_robot::setBaseCommand()
{
    if(!kortex_robot::getFeedback())
    {
        return false;
    }

    for(int i =0;i<q.size();i++)
    {
        base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
    }
    return true;
}

//Sends the next actuator position
//Assumes default control mode of position
//Need to be called after refreshFeedback and after base_command is set
bool kortex_robot::sendPosition(const Eigen::VectorXd& desired_q_step)
{
    if(!low_level_servoing)
    {
        return false;
    }
    try
    {
        //Get new frame ID
        base_command.set_frame_id(base_command.frame_id() + 1);;
        if(base_command.frame_id()>65535)
        {
            base_command.set_frame_id(0);
        }
        //Set new desired position and id for new command
        for(int i =0; i<q.size(); i++)
        {   
            base_command.mutable_actuators(i)->set_position(fmod(desired_q_step(i),360.0));
            base_command.mutable_actuators(i)->set_command_id(base_command.frame_id());
        }
        base_feedback = base_cyclic->Refresh(base_command,0);
        return true;
    }
    catch(k_api::KDetailedException& ex)
    {
        kExceptionHandle(ex);
        return false;
    }
    catch(std::runtime_error& ex2)
    {
        stdExceptionHandle(ex2);
        return false;
    }
}