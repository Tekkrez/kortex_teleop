import rclpy
from rclpy.node import Node
from rclpy.task import Future
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

class validate_model(Node):
    def __init__(self):
        super().__init__("cgn_tester")

        self.cv_bridge = CvBridge()

        self.cam_info_sub = self.create_subscription(CameraInfo,"/head/right_camera/color/camera_info",self.set_camera_info,1)

        self.client = self.create_client(srv_type=GraspReq,srv_name='request_grasp')
        self.future: Future = None

    def set_camera_info(self,msg: CameraInfo):
        
        self.camera_info_msg = msg

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.client.srv_name} service not available, waiting...')

        request = GraspReq.Request()
        self.segmentation_map, self.rgb_image, self.depth_image, self.k_intrinsics, pc_full, pc_colors = load_available_input_data('/home/tekkrez/code_repo/contact_graspnet_pytorch/test_data/7.npy', K=None)
    
        # cv2.imshow('pic3',self.depth_image)
        # # cv2.imshow('pic2',self.rgb_image)
        # cv2.waitKey(0)
        # # closing all open windows
        # cv2.destroyAllWindows()
        self.colour_msg = self.cv_bridge.cv2_to_imgmsg(self.rgb_image,encoding='rgb8')
        self.depth_msg = self.cv_bridge.cv2_to_imgmsg(self.depth_image,encoding="32FC1")
        self.segmentation_map_msg = self.cv_bridge.cv2_to_imgmsg(self.segmentation_map)
        # Create request
        request.image = self.colour_msg
        request.depth_image = self.depth_msg
        request.camera_info = self.camera_info_msg
        request.segmentation_map = self.segmentation_map_msg
        # Send service request
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.process_response)

    def process_response(self, future: Future):
        response = future.result()
        if response.success:
            print(f'grasp request processed')

def main(args=None):
    rclpy.init(args=args)
    grasp_gen_node = validate_model()
    rclpy.spin(grasp_gen_node)

    grasp_gen_node.destroy_node()
    rclpy.shutdown()

    