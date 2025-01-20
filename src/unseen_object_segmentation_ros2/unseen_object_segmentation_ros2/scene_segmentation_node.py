import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge,CvBridgeError

import torch
import torch.nn.parallel
import torch.backends.cudnn as cudnn
import torch.nn

import threading

import cv2
import numpy as np

from utils.blob import pad_im
from utils.mask import visualize_segmentation
import networks
from fcn.config import cfg,cfg_from_file, get_output_dir
from fcn.test_dataset import test_sample
lock = threading.Lock()

def compute_xyz(depth_img, fx, fy, px, py, height, width):
    indices = np.indices((height, width), dtype=np.float32).transpose(1,2,0)
    z_e = depth_img
    x_e = (indices[..., 1] - px) * z_e / fx
    y_e = (indices[..., 0] - py) * z_e / fy
    xyz_img = np.stack([x_e, y_e, z_e], axis=-1) # Shape: [H x W x 3]
    return xyz_img

class scene_segmenter(Node):
    def __init__(self):
        super().__init__('scene_segmenter')
        # Properties
        self.im = None
        self.im_scale = 1.0
        self.message_slop = 0.06
        self.depth = None
        self.rgb_frameid = None
        self.rgb_framestamp = None
        self.rgb_time = None
        self.depth_time = None

        self.camera_intrinsics_set = False
        self.fx=None
        self.fy=None
        self.px=None
        self.py=None

        self.network = None
        self.network_crop = None
        

        # ROS2 Parameters
        self.declare_parameter("network","seg_resnet34_8s_embedding")
        # FIXME:
        self.declare_parameter("pretrained_checkpoint","/home/tekkrez/kortex_teleop/build/unseen_object_segmentation_ros2/data/checkpoints/seg_resnet34_8s_embedding_cosine_rgbd_add_sampling_epoch_16.checkpoint.pth")
        self.declare_parameter("pretrained_crop_checkpoint","/home/tekkrez/kortex_teleop/build/unseen_object_segmentation_ros2/data/checkpoints/seg_resnet34_8s_embedding_cosine_rgbd_add_crop_sampling_epoch_16.checkpoint.pth")
        self.declare_parameter("cfgs","/home/tekkrez/kortex_teleop/build/unseen_object_segmentation_ros2/experiments/cfgs/seg_resnet34_8s_embedding_cosine_rgbd_add_tabletop.yml")

        # Subscribers
        self.cam_info_sub = self.create_subscription(CameraInfo,"/camera/camera/color/camera_info",self.set_camera_info,1)
        self.colour_sub = self.create_subscription(Image,"/camera/camera/color/image_raw",self.colour_callback,3)
        self.depth_sub = self.create_subscription(Image,"/camera/camera/aligned_depth_to_color/image_raw",self.depth_callback,3)
        # self.colour_sub = message_filters.Subscriber(self,"/camera/color/image_raw",Image)
        # self.depth_sub = message_filters.Subscriber(self,"/camera/depth_registered/image_rect",Image)

        # Publishers
        self.label_pub = self.create_publisher(Image,"seg_image_label",3)
        self.refined_label_pub = self.create_publisher(Image,"seg_refined_image_label",3)
        self.seg_image_pub = self.create_publisher(Image,"seg_image",3)
        self.seg_refined_image_pub = self.create_publisher(Image,"seg_refined_image",3)

        #CV Bride
        self.cv_bridge = CvBridge()

        # # Synchronized callback
        # time_sync = message_filters.ApproximateTimeSynchronizer([self.colour_sub,self.depth_sub],queue_size=1,slop=0.05)
        # time_sync.registerCallback(self.rgbd_callback)
        
        # Setup Segmentation Network configs
        cfg_from_file(self.get_parameter('cfgs').get_parameter_value().string_value)
        cfg.gpu_id = 0
        cfg.device = torch.device('cuda:{:d}'.format(cfg.gpu_id))
        cfg.instance_id = 0
        num_classes = 2
        cfg.MODE = 'TEST'
        cfg.TEST.VISUALIZE = False

        # Setup network
        network_data = torch.load(self.get_parameter('pretrained_checkpoint').get_parameter_value().string_value)
        self.network = networks.__dict__[self.get_parameter('network').get_parameter_value().string_value](num_classes, cfg.TRAIN.NUM_UNITS, network_data).cuda(device=cfg.device)
        self.network = torch.nn.DataParallel(self.network, device_ids=[0]).cuda(device=cfg.device)
        cudnn.benchmark = True
        self.network.eval()
        network_data_crop = torch.load(self.get_parameter('pretrained_crop_checkpoint').get_parameter_value().string_value)
        self.network_crop = networks.__dict__[self.get_parameter('network').get_parameter_value().string_value](num_classes, cfg.TRAIN.NUM_UNITS, network_data_crop).cuda(device=cfg.device)
        self.network_crop = torch.nn.DataParallel(self.network_crop, device_ids=[cfg.gpu_id]).cuda(device=cfg.device)
        self.network_crop.eval()

    def set_camera_info(self,msg):
        if(not self.camera_intrinsics_set):
            intrinsics = np.array(msg.k).reshape(3, 3)
            self.fx = intrinsics[0, 0]
            self.fy = intrinsics[1, 1]
            self.px = intrinsics[0, 2]
            self.py = intrinsics[1, 2]

            self.camera_intrinsics_set=True

    def colour_callback(self,msg):
        im = self.cv_bridge.imgmsg_to_cv2(msg,'bgr8')
        im = pad_im(cv2.resize(im, None, None, fx=self.im_scale, fy=self.im_scale, interpolation=cv2.INTER_LINEAR), 16)
        with lock:
            self.im = im.copy()
            self.rgb_frameid = msg.header.frame_id
            self.rgb_framestamp = msg.header.stamp
            self.rgb_time = rclpy.time.Time(seconds=msg.header.stamp.sec,nanoseconds=msg.header.stamp.nanosec)
            # print("rgb time: ",self.rgb_time)
        if(self.depth_time is not None):
            difference = self.rgb_time-self.depth_time
            if difference.nanoseconds*1e-9 < self.message_slop:
                self.run_network()
            # print("diff: " , difference.nanoseconds*1e-9)
        

    def depth_callback(self,msg):
        
        # Convert to meter based encoding
        depth_cv2 = self.cv_bridge.imgmsg_to_cv2(msg).copy().astype(np.float32)
        depth_cv2 /= 1000
        depth_cv2 = pad_im(cv2.resize(depth_cv2, None, None, fx=self.im_scale, fy=self.im_scale, interpolation=cv2.INTER_NEAREST), 16)
        with lock:
            self.depth = depth_cv2.copy()
            self.depth_time = rclpy.time.Time(seconds=msg.header.stamp.sec,nanoseconds=msg.header.stamp.nanosec)
            # print("depth time: " , self.depth_time)

        if(self.rgb_time is not None):
            difference = self.rgb_time-self.depth_time
            if difference.nanoseconds*1e-9 < self.message_slop:
                self.run_network()
            # print("diff: " , difference.nanoseconds*1e-9)
        

    # def rgbd_callback(self,colour_msg,depth_msg):
        
    #     # Convert to meter based encoding
    #     depth_cv2 = self.cv_bridge.imgmsg_to_cv2(depth_msg).copy().astype(np.float32)
    #     depth_cv2 /= 1000

    #     im =self.cv_bridge.imgmsg_to_cv2(colour_msg,'bgr8')

    #     # TODO: Verify padding/rescaling logic
    #     im_scale = 0.25
    #     im = pad_im(cv2.resize(im, None, None, fx=im_scale, fy=im_scale, interpolation=cv2.INTER_LINEAR), 16)
    #     depth_cv2 = pad_im(cv2.resize(depth_cv2, None, None, fx=im_scale, fy=im_scale, interpolation=cv2.INTER_NEAREST), 16)

    #     with lock:
    #         self.im = im.copy()
    #         self.depth = depth_cv2.copy()
    #         self.rgb_frameid = colour_msg.header.frame_id
    #         self.rgb_framestamp = colour_msg.header.stamp
        
    #     self.run_network()
        
    def run_network(self):
        
        with lock:
            im_color = self.im.copy()
            depth_img = self.depth.copy()
            rgb_frame_id = self.rgb_frameid
            rgb_frame_stamp = self.rgb_framestamp

        # Prep colour image
        im = im_color.astype(np.float32)
        im_tensor = torch.from_numpy(im) / 255.0
        pixel_mean = torch.tensor(cfg.PIXEL_MEANS / 255.0).float()
        im_tensor -= pixel_mean
        image_blob = im_tensor.permute(2, 0, 1)
        sample = {'image_color': image_blob.unsqueeze(0)}

        # Prep depth image
        height = im_color.shape[0]
        width = im_color.shape[1]
        depth_img[np.isnan(depth_img)] = 0
        xyz_img = compute_xyz(depth_img, self.fx, self.fy, self.px, self.py, height, width)
        depth_blob = torch.from_numpy(xyz_img).permute(2, 0, 1)
        sample['depth'] = depth_blob.unsqueeze(0)

        # Run the network
        out_label,out_label_refined = test_sample(sample, self.network, self.network_crop)

        # publish segmentation mask
        label = out_label[0].cpu().numpy()
        label_msg = self.cv_bridge.cv2_to_imgmsg(label.astype(np.uint8))
        label_msg.header.stamp = rgb_frame_stamp
        label_msg.header.frame_id = rgb_frame_id
        label_msg.encoding = 'mono8'
        self.label_pub.publish(label_msg)

        num_object = len(np.unique(label)) - 1
        print('%d objects' % (num_object))

        if out_label_refined is not None:
            label_refined = out_label_refined[0].cpu().numpy()
            label_msg_refined = self.cv_bridge.cv2_to_imgmsg(label_refined.astype(np.uint8))
            label_msg_refined.header.stamp = rgb_frame_stamp
            label_msg_refined.header.frame_id = rgb_frame_id
            label_msg_refined.encoding = 'mono8'
            self.refined_label_pub.publish(label_msg_refined)

        # publish segmentation images
        im_label = visualize_segmentation(im_color[:, :, (2, 1, 0)], label, return_rgb=True)
        rgb_msg = self.cv_bridge.cv2_to_imgmsg(im_label, 'rgb8')
        rgb_msg.header.stamp = rgb_frame_stamp
        rgb_msg.header.frame_id = rgb_frame_id
        self.seg_image_pub.publish(rgb_msg)

        if out_label_refined is not None:
            im_label_refined = visualize_segmentation(im_color[:, :, (2, 1, 0)], label_refined, return_rgb=True)
            rgb_msg_refined = self.cv_bridge.cv2_to_imgmsg(im_label_refined, 'rgb8')
            rgb_msg_refined.header.stamp = rgb_frame_stamp
            rgb_msg_refined.header.frame_id = rgb_frame_id
            self.seg_refined_image_pub.publish(rgb_msg_refined)

def main(args=None):
    rclpy.init(args=args)

    scene_segmenter_node = scene_segmenter()
    rclpy.spin(scene_segmenter_node)

    scene_segmenter_node.destroy_node()
    rclpy.shutdown()
