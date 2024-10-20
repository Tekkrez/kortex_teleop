#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.hpp>

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
        Eigen::Isometry3d right_rel_pose;

        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr VR_pose_sub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr hmd_pose_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_pose_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_rel_pose_pub;

        void VRPose_callback(const geometry_msgs::msg::PoseArray& msg)
        {
            Eigen::Isometry3d hmd_pose,right_pose;
            tf2::fromMsg(msg.poses[device::HEAD],hmd_pose);
            tf2::fromMsg(msg.poses[device::RIGHT],right_pose);
            if(!set_ref_point){
                //Reset flag
                set_ref_point = true;
                //Find relative position of shoulder using hmd orientation assuming looking forward
                //Linear corresponds to the rotation
                const Eigen::Vector3d shoulder_adjust(-0.12,-0.7,-0.20);
                shoulder_ref.translation() = hmd_pose.translation()+hmd_pose.rotation()*shoulder_adjust;
                shoulder_ref.linear() = hmd_pose.rotation();
            }
            //Define pose wrt initial shoulder point
            right_rel_pose = shoulder_ref.inverse()*right_pose;
            auto right_rel_pose_stamped = geometry_msgs::msg::PoseStamped();
            right_rel_pose_stamped.header.stamp = this->now();
            right_rel_pose_stamped.header.frame_id = "vr_world";
            right_rel_pose_stamped.pose = tf2::toMsg(right_rel_pose);
            auto hmd_pose_stamped = geometry_msgs::msg::PoseStamped();
            hmd_pose_stamped.header.stamp = this->now();
            hmd_pose_stamped.header.frame_id = "vr_world";
            hmd_pose_stamped.pose = msg.poses[device::HEAD];
            auto right_pose_stamped = geometry_msgs::msg::PoseStamped();
            right_pose_stamped.header.stamp = this->now();
            right_pose_stamped.header.frame_id = "vr_world";
            right_pose_stamped.pose = msg.poses[device::RIGHT];

            hmd_pose_pub->publish(hmd_pose_stamped);
            right_pose_pub->publish(right_pose_stamped);
            right_rel_pose_pub->publish(right_rel_pose_stamped);
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
            right_rel_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("right_rel_pose",5);
        }   

};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<VRManager>());
    rclcpp::shutdown();
    return 0;
}
