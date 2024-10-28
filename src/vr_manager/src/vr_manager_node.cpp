#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vr_manager_utils.h>
#include <chrono>
#include <std_msgs/msg/float64.hpp>

using std::placeholders::_1;

enum device {
    HEAD = 0,
    RIGHT = 1,
    LEFT = 2
};

class VRManager : public rclcpp::Node
{
    private:
        bool set_ref_point = false;
        Eigen::Isometry3d shoulder_ref;
        // Eigen::Isometry3d right_rel_pose;
        Eigen::Quaterniond right_rel_quaternion;
        LowPassFilter pos_filter;
        LowPassFilter quat_filter;
        std::chrono::high_resolution_clock::time_point time_point;
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr VR_pose_sub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr hmd_pose_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_pose_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_rel_pose_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_shoulder_pose_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr delta_time_pub;
        Eigen::Quaterniond ee_rot_adjustment;
        Eigen::Vector3d ee_pos_adjustment,shoulder_adjust;
        double user_arm_length,robot_arm_length;
        void VRPose_callback(const geometry_msgs::msg::PoseArray& msg)
        {
            Eigen::Isometry3d hmd_pose,right_pose,right_rel_pose;
            tf2::fromMsg(msg.poses[device::HEAD],hmd_pose);
            tf2::fromMsg(msg.poses[device::RIGHT],right_pose);
            //adjust rotation to align with URDF
            right_pose.linear() = (Eigen::Quaterniond(right_pose.rotation())*ee_rot_adjustment).toRotationMatrix();
            if(!set_ref_point){
                //Reset flag
                set_ref_point = true;
                //Find relative position of shoulder using hmd orientation assuming looking forward
                //Linear corresponds to the rotation
                shoulder_ref.translation() = hmd_pose.translation()+hmd_pose.rotation()*shoulder_adjust;
                shoulder_ref.linear() = hmd_pose.rotation();
                right_rel_pose = shoulder_ref.inverse()*right_pose;
                right_rel_quaternion = Eigen::Quaterniond(right_rel_pose.rotation());
                //TODO: Setup alpha as param
                quat_filter.lowPassFilterInit(right_rel_quaternion.coeffs(),0.2);
                pos_filter.lowPassFilterInit(right_rel_pose.translation(),0.15);
                //Setup time_point
                time_point = std::chrono::high_resolution_clock::now();
            }
            double time_delta = 0.001*std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-time_point).count();
            time_point = std::chrono::high_resolution_clock::now();
            //Define pose wrt initial shoulder point
            right_rel_pose = shoulder_ref.inverse()*right_pose;
            //Check for quaternion flipping with prev quaternion
            if(Eigen::Quaterniond(right_rel_pose.rotation()).dot(right_rel_quaternion)<0)
            {
                Eigen::Quaterniond new_quat(right_rel_pose.rotation());
                right_rel_quaternion = Eigen::Quaterniond(Eigen::Vector4d(quat_filter.applyFilterTimeScaled(-1*new_quat.coeffs(),time_delta,0.05)));
                right_rel_quaternion.normalize();
            }
            else
            {
                Eigen::Quaterniond new_quat(right_rel_pose.rotation());
                right_rel_quaternion  = Eigen::Quaterniond(Eigen::Vector4d(quat_filter.applyFilterTimeScaled(new_quat.coeffs(),time_delta,0.05)));
                right_rel_quaternion.normalize();
            }
            auto right_rel_pose_stamped = geometry_msgs::msg::PoseStamped();
            right_rel_pose_stamped.header.stamp = this->now();
            right_rel_pose_stamped.header.frame_id = "vr_world";
            //Get vector3d of position relative to shoulder
            //Adjust shoulder position relative to robot
            //Scale desired position
            //Apply filter and publish
            Eigen::Vector3d desired_position;
            desired_position = pos_filter.applyFilterTimeScaled(right_rel_pose.translation(),time_delta);
            desired_position = desired_position*robot_arm_length/user_arm_length+ee_pos_adjustment;
            right_rel_pose_stamped.pose.position = tf2::toMsg(desired_position);
            right_rel_pose_stamped.pose.orientation = tf2::toMsg(right_rel_quaternion);
            auto right_shoulder_pose_stamped = geometry_msgs::msg::PoseStamped();
            right_shoulder_pose_stamped.header.stamp = this->now();
            right_shoulder_pose_stamped.header.frame_id = "vr_world";
            right_shoulder_pose_stamped.pose = tf2::toMsg(shoulder_ref);
            auto hmd_pose_stamped = geometry_msgs::msg::PoseStamped();
            hmd_pose_stamped.header.stamp = this->now();
            hmd_pose_stamped.header.frame_id = "vr_world";
            hmd_pose_stamped.pose = msg.poses[device::HEAD];
            auto right_pose_stamped = geometry_msgs::msg::PoseStamped();
            right_pose_stamped.header.stamp = this->now();
            right_pose_stamped.header.frame_id = "vr_world";
            right_pose_stamped.pose = msg.poses[device::RIGHT];
            auto delta_time_msg = std_msgs::msg::Float64();
            delta_time_msg.data = time_delta;

            hmd_pose_pub->publish(hmd_pose_stamped);
            right_pose_pub->publish(right_pose_stamped);
            right_rel_pose_pub->publish(right_rel_pose_stamped);
            right_shoulder_pose_pub->publish(right_shoulder_pose_stamped);
            delta_time_pub->publish(delta_time_msg);
        }

    public:

        VRManager() : Node("VR_Manager")
        {
            // Pose Subscriber
            rclcpp::QoS sub_qos(rclcpp::KeepLast(1),rmw_qos_profile_sensor_data);
            VR_pose_sub = this->create_subscription<geometry_msgs::msg::PoseArray>("vr_pose",sub_qos,std::bind(&VRManager::VRPose_callback,this,_1));
            //Pose Publisher
            hmd_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("hmd_pose",5);
            right_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("right_pose",5);
            right_rel_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("desired_pose",5);
            right_shoulder_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("right_shoulder_pose",5);
            delta_time_pub = this->create_publisher<std_msgs::msg::Float64>("delta_time",5);
            
            //Shoulder adjust
            shoulder_adjust <<-0.1,-0.07,-0.20;
            //Rotate by 90 deg about Y axis so that end effector frame on the URDF is more naturally aligned with controller pose
            ee_rot_adjustment.vec() << 0,0.7071068,0;
            ee_rot_adjustment.w() = 0.7071068;
            //Raise by 0.28 to align shoulder with robot shoulder
            ee_pos_adjustment << 0,0,0.28;
            user_arm_length = 0.66;
            //bit shoter that actual
            robot_arm_length = 0.85;

        }   
};
int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<VRManager>());
    rclcpp::shutdown();
    return 0;
}
