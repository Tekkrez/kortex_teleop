#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <teleop_interfaces/srv/execute_grasp.hpp>
#include <teleop_interfaces/srv/manipulator_waypoints.hpp>
#include <teleop_interfaces/srv/send_transform.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <thread>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;



class GraspExecutor : public rclcpp::Node
{
    private:
    // Properties
    // Offset from grasp to approach from
    double offset_distance = -0.1;
    double grasp_offset_adjustment = -0.015;
    geometry_msgs::msg::Pose current_pose;
    Eigen::Isometry3d head_colour_wrt_world;
    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    // Subscriber
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr current_pose_sub;
    // Service
    rclcpp::Service<teleop_interfaces::srv::ExecuteGrasp>::SharedPtr grasp_execute_service;
    rclcpp::Service<teleop_interfaces::srv::SendTransform>::SharedPtr send_transform_service;
    // Service client
    rclcpp::Client<teleop_interfaces::srv::ManipulatorWaypoints>::SharedPtr waypoint_service_client;

    void current_pose_callback(const geometry_msgs::msg::Pose& msg)
    {
        this->current_pose = msg;
    }

    void client_response_callback(rclcpp::Client<teleop_interfaces::srv::ManipulatorWaypoints>::SharedFuture future)
    {
        auto response = future.get();
        if(response->recieved)
        {
            std::cout << "Grasp execution has begun" << std::endl;
        }
    }

    Eigen::Isometry3d get_offset_pose(const Eigen::Isometry3d& target_pose, float offset_distance = 0)
    {
        Eigen::Isometry3d offset_pose = target_pose;
        // Apply the offset in the local z direction wrt camera_frame
        Eigen::Vector3d local_offset =  target_pose.rotation()*Eigen::Vector3d::UnitZ()*offset_distance;
        offset_pose.translation() += local_offset;
        // Transform to global frame
        offset_pose = this->head_colour_wrt_world*offset_pose;

        return offset_pose;
    }

    void execute_grasp_callback(const std::shared_ptr<teleop_interfaces::srv::ExecuteGrasp_Request> request, const std::shared_ptr<teleop_interfaces::srv::ExecuteGrasp_Response> response)
    {
        // Create vecs
        std::vector<geometry_msgs::msg::Pose> pose_vec;
        std::vector<bool> gripper_state_vec;
        // Add points
        Eigen::Isometry3d grasp_pose;
        tf2::fromMsg(request->grasp_pose,grasp_pose);
        pose_vec.emplace_back(tf2::toMsg(get_offset_pose(grasp_pose,this->offset_distance)));
        // do it twice to ensure gripper is open
        pose_vec.emplace_back(tf2::toMsg(get_offset_pose(grasp_pose,this->offset_distance)));
        grasp_pose = get_offset_pose(grasp_pose,this->grasp_offset_adjustment);
        // grasp_pose = head_colour_wrt_world*grasp_pose;
        pose_vec.emplace_back(tf2::toMsg(grasp_pose));
        pose_vec.emplace_back(this->current_pose);
        // do it twice to ensure gripper is open
        gripper_state_vec.emplace_back(true);
        gripper_state_vec.emplace_back(false);
        gripper_state_vec.emplace_back(true);
        gripper_state_vec.emplace_back(true);
        // Create and send waypoint request
        auto waypoint_req = std::make_shared<teleop_interfaces::srv::ManipulatorWaypoints::Request>();
        waypoint_req->poses = pose_vec;
        waypoint_req->gripper_state = gripper_state_vec;

        while (!waypoint_service_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the Waypoint service. Exiting.");
                break;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waypoint service not available, waiting again...");
        }
        auto result = waypoint_service_client->async_send_request(waypoint_req,std::bind(&GraspExecutor::client_response_callback,this,std::placeholders::_1));

        response->success = true;
    }

    void send_transform_callback(const std::shared_ptr<teleop_interfaces::srv::SendTransform_Request> request, const std::shared_ptr<teleop_interfaces::srv::SendTransform_Response> response)
    {
        response->transform = tf2::eigenToTransform(this->head_colour_wrt_world);
        response->transform.header.stamp = this->now();
        response->transform.header.frame_id = "world";
        response->transform.child_frame_id = "head_camera_colour_frame";
    }

    public:

    GraspExecutor(): Node("Grasp_Executor")
    {
        // Interfaces
        current_pose_sub = this->create_subscription<geometry_msgs::msg::Pose>("current_ee_pose",1,std::bind(&GraspExecutor::current_pose_callback,this,_1));
        grasp_execute_service = this->create_service<teleop_interfaces::srv::ExecuteGrasp>("execute_grasp",std::bind(&GraspExecutor::execute_grasp_callback,this,_1,_2));
        waypoint_service_client = this->create_client<teleop_interfaces::srv::ManipulatorWaypoints>("manipulator_waypoints");
        // TF listener
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        // Get transform
        bool transform_found = false;
        geometry_msgs::msg::TransformStamped head_colour_wrt_world_msg;
        while(!transform_found)
        {
            try
            {
                if (!rclcpp::ok()) 
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the Waypoint service. Exiting.");
                    break;
                }
                head_colour_wrt_world_msg = tf_buffer->lookupTransform("world","head_camera_colour_frame",tf2::TimePointZero);
                transform_found = true;
                
            }
            catch (const tf2::TransformException & ex)
            {
                RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s","World", "head_camera_colour_frame", ex.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }
        head_colour_wrt_world = tf2::transformToEigen(head_colour_wrt_world_msg);
        // Register service after transform is found
        send_transform_service = this->create_service<teleop_interfaces::srv::SendTransform>("send_transform",std::bind(&GraspExecutor::send_transform_callback,this,_1,_2));
    }
};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<GraspExecutor>());
    rclcpp::shutdown();
    return 0;
}