#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <teleop_interfaces/srv/visualize_grasp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class CameraInterceptor : public rclcpp::Node
{
    private:
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_sub;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr camera_pub;
        rclcpp::Service<teleop_interfaces::srv::VisualizeGrasp>::SharedPtr grasp_view_service;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr head_view_service;
        sensor_msgs::msg::CompressedImage grasp_visualization;
        bool visualize_grasp = false;
        bool ignore_next_grasp = false;
        cv_bridge::CvImagePtr cv_ptr;
        rclcpp::Time time_point;

        void camera_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
        {
            if(visualize_grasp)
            {
                camera_pub->publish(grasp_visualization);
            }
            else
            {
                camera_pub->publish(*msg);
            }
        }

        void grasp_view_callback(const std::shared_ptr<teleop_interfaces::srv::VisualizeGrasp::Request> request, const std::shared_ptr<teleop_interfaces::srv::VisualizeGrasp::Response> response)
        {
            // Convert image to compressed format
            try
            {   std::cout<<"Visualizing Grasp"<<std::endl;
                std::cout<<request->grasp_visualization.encoding<<std::endl;
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(request->grasp_visualization,request->grasp_visualization.encoding);
                std::vector<uchar> compressed_image_data;
                // Convert from bgr to rgb
                cv::cvtColor(cv_ptr->image,cv_ptr->image,cv::COLOR_BGR2RGB); 
                cv::imencode(".jpg",cv_ptr->image,compressed_image_data);            
                // Create compressed Imgage message
                sensor_msgs::msg::CompressedImage grasp_im;
                grasp_im.header = request->grasp_visualization.header;
                grasp_im.format = "jpeg";
                grasp_im.data = compressed_image_data;
                this->grasp_visualization = grasp_im;

                // this->grasp_visualization = request->grasp_visualization;
            }
            catch(const cv_bridge::Exception& e)
            {
                std::cerr << e.what() << '\n';
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }

            // Set flags
            if(time_point + rclcpp::Duration(5,0) < this->now())
            {
                std::cout<< "Time out" <<std::endl;
                ignore_next_grasp = false;
            }
            if(ignore_next_grasp)
            {
                std::cout<< "Ignored" <<std::endl;
                ignore_next_grasp = false;
            }
            else if(request->generated_grasps)
            {
                visualize_grasp = true;
            }
            response->success = true;
        }

        void head_view_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
        {
            if(request->data)
            {
                if(!visualize_grasp)
                {
                    std::cout<< "Ignoring next set of generated grasps" <<std::endl;
                    ignore_next_grasp = true;
                    time_point = this->now();
                }

                std::cout<< "Stop Visualizing grasps" <<std::endl;
                visualize_grasp = false;
            }
            response->success = true;
        }


    public:
        CameraInterceptor() : Node("Camera_Interceptor")
        {
            time_point = this->now();
            camera_sub = this->create_subscription<sensor_msgs::msg::CompressedImage>("/head/right_camera/color/image_raw/compressed",1,std::bind(&CameraInterceptor::camera_callback,this,_1));
            camera_pub = this->create_publisher<sensor_msgs::msg::CompressedImage>("vr_camera_feed",1);
            grasp_view_service = this->create_service<teleop_interfaces::srv::VisualizeGrasp>("set_grasp_view",std::bind(&CameraInterceptor::grasp_view_callback,this,_1,_2));
            head_view_service = this->create_service<std_srvs::srv::SetBool>("set_head_view",std::bind(&CameraInterceptor::head_view_callback,this,_1,_2));
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraInterceptor>());
    rclcpp::shutdown();
    return 0;
}