import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from teleop_interfaces.srv import GraspReq

import numpy as np
import cv2

from contact_graspnet_pytorch.contact_grasp_estimator import GraspEstimator
from contact_graspnet_pytorch import config_utils
from contact_graspnet_pytorch.visualization_utils_o3d import visualize_grasps, show_image
from contact_graspnet_pytorch.checkpoints import CheckpointIO 
from contact_graspnet_pytorch.data import load_available_input_data

import threading
import os

lock = threading.Lock()

class Grasp_Generator(Node):
    def __init__(self):
        super().__init__('grasp_generator')

        #Properties
        self.rgb_image = None
        self.depth_image = None
        self.k_intrinsics = None
        self.segmentation_map = None
        self.checkpoint_dir = None
        self.grasp_network = None
        self.cv_bridge = CvBridge()

        # ROS2 Parameters
        # TODO: Fix parameters
        self.declare_parameter("checkpoint__dir",'/home/tekkrez/kortex_teleop/install/contact_graspnet_pt_ros2/share/contact_graspnet_pt_ros2/checkpoints')
        self.declare_parameter("filter_grasps","true")
        self.checkpoint_dir = self.get_parameter('checkpoint__dir').get_parameter_value().string_value
        # Create service
        self.grasp_service = self.create_service(GraspReq,"request_grasp",self.request_grasp_callback)
        # sample data:
        # self.segmentation_map, self.rgb_image, self.depth_image, self.k_intrinsics, pc_full, pc_colors = load_available_input_data('/home/tekkrez/code_repo/contact_graspnet_pytorch/test_data/7.npy', K=None)
        # cv2.imshow('pic',self.segmentation_map)
        # # cv2.imshow('pic2',self.rgb_image)
        # cv2.waitKey(0)
        # # closing all open windows
        # cv2.destroyAllWindows()

        # Build the network
        global_config = config_utils.load_config(self.checkpoint_dir,batch_size=1)
        print(str(global_config))
        self.grasp_network = GraspEstimator(global_config)
        checkpoint_io = CheckpointIO(checkpoint_dir=self.checkpoint_dir, model=self.grasp_network.model)
        # self.run_network()
        
    def run_network(self):
        pc_full,pc_segments,pc_colours = self.grasp_network.extract_point_clouds(depth = self.depth_image,segmap=self.segmentation_map,segmap_id = 1,rgb=self.rgb_image,K=self.k_intrinsics,skip_border_objects=True,z_range=[0.2,1.8])
        # desired_keys = [1.0,2.0,3.0]
        # pc_segments  = {key: pc_segments[key] for key in desired_keys if key in pc_segments}
        # for i in pc_segments:
        #     print("here")
        #     print(i)
        pred_grasps_cam,scores,contact_pts,_ = self.grasp_network.predict_scene_grasps(pc_full,pc_segments=pc_segments,filter_grasps=True,local_regions=True)
        # Visualize results          
        show_image(self.rgb_image, self.segmentation_map)
        visualize_grasps(pc_full, pred_grasps_cam, scores, plot_opencv_cam=True, pc_colors=pc_colours)

    def request_grasp_callback(self,request: GraspReq.Request,response: GraspReq.Response):
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(request.depth_image).copy().astype(np.float32)/1000
        self.rgb_image = self.cv_bridge.imgmsg_to_cv2(request.image,desired_encoding='rgb8').copy()
        self.segmentation_map = self.cv_bridge.imgmsg_to_cv2(request.segmentation_map).copy()
        
        # cv2.imshow('pic',self.segmentation_map)
        # # cv2.imshow('pic2',self.rgb_image)
        # cv2.waitKey(0)
        # # closing all open windows
        # cv2.destroyAllWindows()

        intrinsics = request.camera_info.k
        self.k_intrinsics = np.array(intrinsics).reshape(3, 3)
        self.run_network()
        response.success = True

        return response

def main(args=None):
    rclpy.init(args=args)
    grasp_gen_node = Grasp_Generator()
    rclpy.spin(grasp_gen_node)

    grasp_gen_node.destroy_node()
    rclpy.shutdown()