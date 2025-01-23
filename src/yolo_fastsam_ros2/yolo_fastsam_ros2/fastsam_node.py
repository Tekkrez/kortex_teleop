import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from ultralytics import FastSAM
from ultralytics.engine.results import Results
from ultralytics.engine.results import Masks

import cv2
import torch
import numpy as np

import threading

lock = threading.Lock()

class instance_segmenter(Node):
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
        self.camera_info_message = None
        self.segmentation_message = None

        # Subscribers
        self.cam_info_sub = self.create_subscription(CameraInfo,"/camera/camera/color/camera_info",self.set_camera_info,1)
        self.colour_sub = self.create_subscription(Image,"/camera/camera/color/image_raw",self.colour_callback,3)
        self.depth_sub = self.create_subscription(Image,"/camera/camera/aligned_depth_to_color/image_raw",self.depth_callback,3)

        # Publishers
        self.segmented_mask_pub = self.create_publisher(Image,"segmented_mask",3)
        self.segmented_image_pub = self.create_publisher(Image,"segmented_image",3)

        # Import network
        self.network = FastSAM("FastSAM-s.pt")

    def set_camera_info(self,msg: CameraInfo):
        with lock:
            self.camera_info_message = msg

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
            self.colour_message = msg
            self.image = im.copy()
            self.rgb_frameid = msg.header.frame_id
            self.rgb_framestamp = msg.header.stamp
            self.rgb_time = rclpy.time.Time(seconds=msg.header.stamp.sec,nanoseconds=msg.header.stamp.nanosec)

        if(self.depth_time is not None):
            difference = self.rgb_time-self.depth_time
            if difference.nanoseconds*1e-9 < self.message_slop:
                self.run_network()

    def run_network(self):
        # ,points = [[self.image.shape[0]/2,self.image.shape[1]/2]]
        center = [600,350]
        # Run segmentation network
        results = self.network(self.image,points = [center],conf=0.2, verbose=False)
        # Process results
        result: Results =  results[0]
        masks: Masks = result.masks
        collapsed_mask = torch.any(masks.data,dim=0).type(torch.uint8)*255
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

        
def main(args=None):
    rclpy.init(args=args)
    instance_segmenter_node = instance_segmenter()
    rclpy.spin(instance_segmenter_node)

    instance_segmenter_node.destroy_node()
    rclpy.shutdown()