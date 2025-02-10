#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vr_manager_utils.h>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster_node.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <teleop_interfaces/srv/manipulator_waypoints.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;

class Camera_Calibrator : public rclcpp::Node
{
    private:
        // Flags
        bool camera_calibrated = false;
        bool movement_initiated = false;
        bool movement_done = false;
        // Camera Calibration
        ros2_aruco_interfaces::msg::ArucoMarkers last_arm_aruco_message;
        ros2_aruco_interfaces::msg::ArucoMarkers last_head_aruco_message;
        rclcpp::Time head_aruco_time;
        rclcpp::Time arm_aruco_time;
        bool head_aruco_msg_recieved = false;
        bool arm_aruco_msg_recieved = false;
        double max_sync_time_gap = 0.06;
        // TF
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_braodcaster;
        // Subscriptions
        rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr head_aruco_marker_sub;
        rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr arm_aruco_marker_sub;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr current_pose_sub;
        // Service client
        rclcpp::Client<teleop_interfaces::srv::ManipulatorWaypoints>::SharedPtr waypoint_service_client;

        // Camera_calibration_point
        Eigen::Isometry3d calibration_pose;
        Eigen::Isometry3d home_position_pose;

        void head_aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers& msg)
        {
            if(!camera_calibrated && movement_done)
            {
                head_aruco_msg_recieved = true;
                this->last_head_aruco_message = msg;
                this->head_aruco_time = rclcpp::Time(msg.header.stamp);
                if(this->arm_aruco_msg_recieved)
                {             
                    rclcpp::Duration time_gap = this->head_aruco_time-this->arm_aruco_time;
                    if(time_gap.seconds()<this->max_sync_time_gap)
                    {
                        publish_head_camera_tf();
                    }
                }
            }
        }
        
        void arm_arcuo_callback(const ros2_aruco_interfaces::msg::ArucoMarkers& msg)
        {
            if(!camera_calibrated && movement_done)
            {
                arm_aruco_msg_recieved = true;
                this->last_arm_aruco_message = msg;
                this->arm_aruco_time = rclcpp::Time(msg.header.stamp);
                if (this->head_aruco_msg_recieved)
                {
                    rclcpp::Duration time_gap = this->arm_aruco_time-this->head_aruco_time;
                    if(time_gap.seconds()<this->max_sync_time_gap)
                    {
                        publish_head_camera_tf();
                    }
                }
            }
        }

        void publish_head_camera_tf()
        {
            // Find id's shared by both markers
            std::vector<long> arm_markers = this->last_arm_aruco_message.marker_ids;
            std::vector<long> head_markers = this->last_head_aruco_message.marker_ids;
            std::vector<long> arm_markers_sorted = arm_markers;
            std::vector<long> head_markers_sorted = head_markers;
            // Sorting to find intersection
            std::sort(arm_markers_sorted.begin(),arm_markers_sorted.end());
            std::sort(head_markers_sorted.begin(),head_markers_sorted.end());
            std::vector<long> common_marker_ids;
            std::set_intersection(arm_markers_sorted.begin(),arm_markers_sorted.end(),head_markers_sorted.begin(),head_markers_sorted.end(),std::back_inserter(common_marker_ids));
            std::cout<< " Common marker IDs : ";
            for(auto id : common_marker_ids)
            {
                std::cout<<id<<" ";
            }
            std::cout<<std::endl;

            geometry_msgs::msg::TransformStamped head_to_colour_frame_msg;
            geometry_msgs::msg::TransformStamped world_to_colour_frame_msg;
            
            try{
                head_to_colour_frame_msg = tf_buffer->lookupTransform("head_camera","head_camera_colour_frame",tf2::TimePointZero);
            }
            catch (const tf2::TransformException & ex) {
                RCLCPP_INFO(
                    this->get_logger(), "Could not transform %s to %s: %s",
                    "head_camera_colour_frame", "head_camera", ex.what());
                return;
            }
            try{
                world_to_colour_frame_msg = tf_buffer->lookupTransform("world","camera_color_frame",tf2::TimePointZero);
            }
            catch (const tf2::TransformException & ex) {
                RCLCPP_INFO(
                    this->get_logger(), "Could not transform %s to %s: %s",
                    "camera_color_frame", "world", ex.what());
                return;
            }

            std::vector<Eigen::Isometry3d> arm_shared_aruco_poses = find_pose_matching_ids(common_marker_ids,arm_markers,this->last_arm_aruco_message);
            std::vector<Eigen::Isometry3d> head_shared_aruco_poses = find_pose_matching_ids(common_marker_ids,head_markers,this->last_head_aruco_message);

            Eigen::Isometry3d best_aruco_pose_from_arm = find_best_pose(arm_shared_aruco_poses);
            Eigen::Isometry3d best_aruco_pose_from_head = find_best_pose(head_shared_aruco_poses);


            Eigen::Isometry3d head_to_colour_frame = tf2::transformToEigen(head_to_colour_frame_msg);
            Eigen::Isometry3d world_to_colour_frame = tf2::transformToEigen(world_to_colour_frame_msg);
            Eigen::Isometry3d head_to_aruco = head_to_colour_frame*best_aruco_pose_from_head;
            Eigen::Isometry3d world_to_aruco = world_to_colour_frame*best_aruco_pose_from_arm;
            Eigen::Isometry3d world_to_head = world_to_aruco*head_to_aruco.inverse();

            std::vector<geometry_msgs::msg::TransformStamped> transform_messages;
            geometry_msgs::msg::TransformStamped world_to_head_msg = tf2::eigenToTransform(world_to_head);
            world_to_head_msg.header.stamp = this->get_clock()->now();
            world_to_head_msg.header.frame_id = "world";
            world_to_head_msg.child_frame_id = "head_camera";
            transform_messages.emplace_back(world_to_head_msg);

            world_to_colour_frame_msg.child_frame_id = "test_123";
            transform_messages.emplace_back(world_to_colour_frame_msg);

            geometry_msgs::msg::TransformStamped world_to_arm_aruco_msg = tf2::eigenToTransform(world_to_aruco);
            world_to_arm_aruco_msg.header.stamp = this->get_clock()->now();
            world_to_arm_aruco_msg.header.frame_id = "world";
            world_to_arm_aruco_msg.child_frame_id = "world_arm_aruco_marker";
            transform_messages.emplace_back(world_to_arm_aruco_msg);

            geometry_msgs::msg::TransformStamped arm_to_aruco_msg = tf2::eigenToTransform(best_aruco_pose_from_arm);
            arm_to_aruco_msg.header.stamp = this->get_clock()->now();
            arm_to_aruco_msg.header.frame_id = "camera_color_frame";
            arm_to_aruco_msg.child_frame_id = "arm_aruco_marker";
            transform_messages.emplace_back(arm_to_aruco_msg);

            geometry_msgs::msg::TransformStamped head_to_aruco_msg = tf2::eigenToTransform(head_to_aruco);
            head_to_aruco_msg.header.stamp = this->get_clock()->now();
            head_to_aruco_msg.header.frame_id = "head_camera_colour_frame";
            head_to_aruco_msg.child_frame_id = "head_aruco_marker";
            transform_messages.emplace_back(head_to_aruco_msg);

            this->tf_static_braodcaster->sendTransform(transform_messages);
            this->camera_calibrated = true;
        }

        void client_response_callback(rclcpp::Client<teleop_interfaces::srv::ManipulatorWaypoints>::SharedFuture future)
        {
            auto response = future.get();
            if(response->recieved)
            {
                std::cout << "Calibration Movement Initiated" << std::endl;
                movement_initiated = true;
            }
        }

        void current_pose_callback(const geometry_msgs::msg::Pose& msg)
        {
            Eigen::Isometry3d current_pose;
            tf2::fromMsg(msg,current_pose);
            Eigen::Quaterniond current_orientation(current_pose.rotation());
            Eigen::Quaterniond target_orientation(calibration_pose.rotation());
            Eigen::AngleAxisd orientatation_error(target_orientation*current_orientation.inverse());
            Eigen::Vector3d position_error(calibration_pose.translation()-current_pose.translation());
            if((position_error.norm()<0.005) && std::abs(orientatation_error.angle())<0.05 && !movement_done)
            {
                std::cout << "Calibration pose target reached" << std::endl;
                movement_done = true;
            }
        }

    public:

        Camera_Calibrator() : Node("VR_Manager")
        {
            //Camera initial setupwaypoint_service_client
            head_aruco_marker_sub = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("head/aruco_markers",1,std::bind(&Camera_Calibrator::head_aruco_callback,this,_1));
            arm_aruco_marker_sub = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("arm/aruco_markers",1,std::bind(&Camera_Calibrator::arm_arcuo_callback,this,_1));
            current_pose_sub = this->create_subscription<geometry_msgs::msg::Pose>("current_ee_pose",1,std::bind(&Camera_Calibrator::current_pose_callback,this,_1));
            waypoint_service_client = this->create_client<teleop_interfaces::srv::ManipulatorWaypoints>("manipulator_waypoints");

            tf_static_braodcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            calibration_pose.translation() << 0.4,0,0.7;
            home_position_pose.translation() << 0.58,0,0.44;
            Eigen::Quaterniond calibration_orientation;
            Eigen::Quaterniond home_position_orientation;
            calibration_orientation.w() = 0.05;
            calibration_orientation.vec()<<0.68,0.72,0;
            calibration_pose.linear() << calibration_orientation.normalized().toRotationMatrix();
            home_position_orientation.w() = 0.5;
            home_position_orientation.vec()<<0.5,0.5,0.5;
            home_position_pose.linear() << home_position_orientation.normalized().toRotationMatrix();
            std::cout<< "Target Position: " << calibration_pose.translation() <<std::endl;
            std::cout<< "Target Orientation: " << Eigen::Quaterniond(calibration_pose.rotation())<<std::endl;
            std::vector<geometry_msgs::msg::Pose> pose_vec;
            std::vector<bool> gripper_state_vec;
            pose_vec.emplace_back(tf2::toMsg(calibration_pose));
            gripper_state_vec.emplace_back(false);
            // Wait for camera to catch a glimpse of the aruco markers
            pose_vec.emplace_back(tf2::toMsg(calibration_pose));
            gripper_state_vec.emplace_back(true);
            // Go to home position
            pose_vec.emplace_back(tf2::toMsg(home_position_pose));
            gripper_state_vec.emplace_back(false);

            auto request = std::make_shared<teleop_interfaces::srv::ManipulatorWaypoints::Request>();
            request->poses = pose_vec;
            request->gripper_state = gripper_state_vec;

            while (!waypoint_service_client->wait_for_service(500ms)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    break;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }

            auto result = waypoint_service_client->async_send_request(request,std::bind(&Camera_Calibrator::client_response_callback,this,std::placeholders::_1));

            //TF listener
            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        }   
};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Camera_Calibrator>());
    rclcpp::shutdown();
    return 0;
}
