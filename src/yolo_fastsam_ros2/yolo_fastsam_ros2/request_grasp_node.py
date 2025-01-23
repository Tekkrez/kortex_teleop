import rclpy
from rclpy.node import Node
from rclpy.task import Future
import rclpy.time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_srvs.srv import SetBool
from teleop_interfaces.srv import GraspReq

from ultralytics import FastSAM
from ultralytics.engine.results import Results
from ultralytics.engine.results import Masks

import cv2
import torch
import numpy as np

import threading

lock = threading.Lock()

class grasp_requester(Node):
    def __init__(self):
        super().__init__('instance_segmenter')
        # Properties
        self.image = None
        self.cv_bridge = CvBridge()
        # Properties for syncing depth and colour images
        self.message_slop = 0.05
        self.colour_time = None
        self.depth_time = None
        # Properties to create service request
        self.rgb_frameid = None
        self.rgb_framestamp = None
        self.rgb_time = None
        self.depth_msg = None
        self.colour_msg = None
        self.camera_info_msg = None
        self.grasp_requested = False

        # Subscribers
        self.cam_info_sub = self.create_subscription(CameraInfo,"/camera/camera/color/camera_info",self.set_camera_info,1)
        self.colour_sub = self.create_subscription(Image,"/camera/camera/color/image_raw",self.colour_callback,3)
        self.depth_sub = self.create_subscription(Image,"/camera/camera/aligned_depth_to_color/image_raw",self.depth_callback,3)

        # Publishers
        self.segmented_mask_pub = self.create_publisher(Image,"segmented_mask",3)
        self.segmented_image_pub = self.create_publisher(Image,"segmented_image",3)
        
        # Service server
        self.server = self.create_service(srv_type=SetBool,srv_name='trigger_grasp',callback=self.trigger_grasp_callback)
        # Service client
        self.client = self.create_client(GraspReq,'request_grasp')
        self.future : Future = None

        # Import network
        self.network = FastSAM("FastSAM-s.pt")

    def set_camera_info(self,msg: CameraInfo):
        with lock:
            self.camera_info_msg = msg

    def depth_callback(self,msg: Image):
        with lock:
            self.depth_msg = msg
            self.depth_time = rclpy.time.Time(seconds=msg.header.stamp.sec,nanoseconds=msg.header.stamp.nanosec)

        if(self.rgb_time is not None):
            difference = self.rgb_time-self.depth_time
            if difference.nanoseconds*1e-9 < self.message_slop:
                self.run_network()

    def colour_callback(self,msg):
        im = self.cv_bridge.imgmsg_to_cv2(msg,'rgb8')

        with lock:
            self.colour_msg = msg
            self.image = im.copy()
            self.rgb_frameid = msg.header.frame_id
            self.rgb_framestamp = msg.header.stamp
            self.rgb_time = rclpy.time.Time(seconds=msg.header.stamp.sec,nanoseconds=msg.header.stamp.nanosec)

        if(self.depth_time is not None):
            difference = self.rgb_time-self.depth_time
            if difference.nanoseconds*1e-9 < self.message_slop:
                self.run_network()

    def trigger_grasp_callback(self,request: SetBool.Request,response: SetBool.Response):
        
        if request.data == True:
            self.grasp_requested = True
        response.success = True
        response.message = "Grasp requested"

        return response

    def run_network(self):
        # ,points = [[self.image.shape[0]/2,self.image.shape[1]/2]]
        center = [600,350]
        results = self.network(self.image,points = [center],conf=0.2)
        result: Results =  results[0]
        masks: Masks = result.masks
        collapsed_mask = torch.any(masks.data,dim=0).type(torch.uint8)*1
        segmented_mask = cv2.resize(collapsed_mask.cpu().numpy(),dsize=(result.orig_shape[1],result.orig_shape[0]))
        masked_img = np.where(segmented_mask[...,None],np.array([0,0,255], dtype='uint8'),self.image)
        segmented_image = cv2.addWeighted(self.image,0.7,masked_img,0.3,0)
        segmented_image = cv2.circle(segmented_image,center=tuple(center),radius=10,color=(0,0,255),thickness=2)
    
        segmented_image_msg = self.cv_bridge.cv2_to_imgmsg(segmented_image,'rgb8')
        segmented_image_msg.header.stamp = self.rgb_framestamp
        segmented_image_msg.header.frame_id = self.rgb_frameid
        self.segmented_image_pub.publish(segmented_image_msg)

        segmented_mask_msg = self.cv_bridge.cv2_to_imgmsg(segmented_mask)
        segmented_mask_msg.header.stamp = self.rgb_framestamp
        segmented_mask_msg.header.frame_id = self.rgb_frameid
        segmented_mask_msg.encoding = 'mono8'
        self.segmented_mask_pub.publish(segmented_mask_msg)

        torch.cuda.empty_cache()
        # Request grasp
        if self.grasp_requested:
            self.grasp_requested = False
            request = GraspReq.Request()
            while not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{self.client.srv_name} service not available, waiting...')
            
            # Create request
            request.image = self.colour_msg
            request.depth_image = self.depth_msg
            request.camera_info = self.camera_info_msg
            request.segmentation_map = segmented_mask_msg
            # Send service request
            self.future = self.client.call_async(request)
            self.future.add_done_callback(self.process_response)

    def process_response(self, future: Future):
        response = future.result()
        if response.success:
            print(f'grasp request processed')

def main(args=None):
    rclpy.init(args=args)
    grasp_requester_node = grasp_requester()
    rclpy.spin(grasp_requester_node)

    grasp_requester_node.destroy_node()
    rclpy.shutdown()