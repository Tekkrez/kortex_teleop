import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from teleop_interfaces.srv import GraspReq
from teleop_interfaces.srv import SendTransform
from geometry_msgs.msg import Pose, Point

import numpy as np
import cv2
import tf_transformations
import torch

from contact_graspnet_pytorch.contact_grasp_estimator import GraspEstimator
from contact_graspnet_pytorch.config_utils import load_config
from contact_graspnet_pytorch.visualization_utils_o3d import visualize_grasps,modified_visualize_grasps
from contact_graspnet_pytorch.checkpoints import CheckpointIO

import threading
import time

lock = threading.Lock()

def pose_matrix_to_msg(matrix):
    translation = matrix[:3,3]
    quaternion = tf_transformations.quaternion_from_matrix(matrix)
    pose_msg = Pose()
    pose_msg.position.x = float(translation[0])
    pose_msg.position.y = float(translation[1])
    pose_msg.position.z = float(translation[2])
    pose_msg.orientation.x = float(quaternion[0])
    pose_msg.orientation.y = float(quaternion[1])
    pose_msg.orientation.z = float(quaternion[2])
    pose_msg.orientation.w = float(quaternion[3])
    return pose_msg

def transform_msg_to_pose_matrix(transform):
    translation = np.array([transform.translation.x,transform.translation.y,transform.translation.z])
    quaternion = np.array([transform.rotation.x,transform.rotation.y,transform.rotation.z,transform.rotation.w])
    matrix = tf_transformations.quaternion_matrix(quaternion)
    matrix[:3,3] = translation
    return matrix

def point_array_to_msg(array):
    point_msg = Point()
    point_msg.x = float(array[0])
    point_msg.y = float(array[1])
    point_msg.z = float(array[2])
    return point_msg


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
        # Network Parameters
        self.local_regions = True
        self.filter_grasps = True
        self.skip_border_objects = False
        self.forward_passes = 1
        # Results
        self.pred_grasps = None
        self.scores = None
        self.contact_pts = None
        self.gripper_openings = None
        self.grasp_viz = None

        # ROS2 Parameters
        # TODO: Fix parameters
        self.declare_parameter("checkpoint__dir",'/home/tekkrez/kortex_teleop/install/contact_graspnet_pt_ros2/share/contact_graspnet_pt_ros2/checkpoints')
        self.declare_parameter("filter_grasps","true")
        self.checkpoint_dir = self.get_parameter('checkpoint__dir').get_parameter_value().string_value

        # Get transform
        self.get_transform_client = self.create_client(SendTransform,'send_transform')
        transform_request = SendTransform.Request()
        while not self.get_transform_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.get_transform_client.srv_name} service not available, waiting...')
        self.transform_future = self.get_transform_client.call_async(transform_request)
        self.transform_future.add_done_callback(self.process_transform_response)

        # Build the network
        global_config = load_config(self.checkpoint_dir,batch_size=1)
        print(str(global_config))
        self.grasp_network = GraspEstimator(global_config)
        checkpoint_io = CheckpointIO(checkpoint_dir=self.checkpoint_dir, model=self.grasp_network.model)
        try:
            load_dict = checkpoint_io.load('model.pt')
        except FileExistsError:
            print('No model checkpoint found')
            load_dict = {}

        # Create service
        self.grasp_service = self.create_service(GraspReq,"request_grasp",self.request_grasp_callback)
        
    def run_network(self):
        pc_full,pc_segments,pc_colours = self.grasp_network.extract_point_clouds(depth = self.depth_image,segmap=self.segmentation_map,segmap_id = 1,rgb=self.rgb_image,K=self.k_intrinsics,skip_border_objects=self.skip_border_objects,z_range=[0.2,1.8])

        self.pred_grasps,self.scores,self.contact_pts,self.gripper_openings = self.grasp_network.predict_scene_grasps(pc_full,pc_segments=pc_segments,filter_grasps=self.filter_grasps,local_regions=self.local_regions,forward_passes=self.forward_passes)

        # show_image(self.rgb_image, self.segmentation_map)
        # visualize_grasps(pc_full, self.pred_grasps, self.scores, plot_opencv_cam=False, pc_colors=pc_colours)
        # visualize the filtered grasps
        self.pred_grasps[1.0], good_pose_index = self.filter_grasps_by_pose(self.pred_grasps[1.0])
        self.scores[1.0] = self.scores[1.0][good_pose_index]
        self.contact_pts[1.0] = self.contact_pts[1.0][good_pose_index]
        self.gripper_openings[1.0] = self.gripper_openings[1.0][good_pose_index]
        self.grasp_viz = modified_visualize_grasps(pc_full, self.pred_grasps, self.scores, plot_opencv_cam=True, pc_colors=pc_colours,intrinsics=self.k_intrinsics)
        # visualize_grasps(pc_full, self.pred_grasps, self.scores, plot_opencv_cam=False, pc_colors=pc_colours)

        # Only take the results for the segmented element
        self.pred_grasps = self.pred_grasps[1.0]
        self.scores = self.scores[1.0]
        self.contact_pts = self.contact_pts[1.0]
        self.gripper_openings = self.gripper_openings[1.0]

    def process_transform_response(self,future):
        response = future.result()
        self.world_to_head_camera = transform_msg_to_pose_matrix(response.transform.transform)

    def filter_grasps_by_pose(self,poses):
        #Filter grasps that are harder to reach for the robot
        print(f'CAMERA TRANSFORM SHAPE: {self.world_to_head_camera.shape}')
        print(f'POSES SHAPE: {poses.shape}')
        poses_in_world_frame = np.matmul(self.world_to_head_camera,poses)
        good_poses_index = np.zeros(poses.shape[0],dtype=bool)
        for i in range(poses.shape[0]):
            pose = poses_in_world_frame[i]
            z_axis = pose[:3,2]
            position = pose[:3,3]
            unit_position = position/np.linalg.norm(position)
            unit_position[2] = 0
            angle_between = np.arccos(np.dot(z_axis,unit_position))
            # Keep only poses that are facing away from the robot with some margin
            if(angle_between<=(np.pi/2+0.3)):
                good_poses_index[i] = True
        # Return back to local frame
        good_poses = poses[good_poses_index]
        print(f'GOOD POSES SHAPE: {good_poses.shape}')
        return good_poses,good_poses_index
    

    def sort_results(self):
        # Find sorted indicies and keep top 20
        sorted_indicies = np.argsort(self.scores)[::-1]
        if(len(self.scores)>20):
            sorted_indicies = sorted_indicies[:20]
        # Apply sort
        self.scores = self.scores[sorted_indicies]
        self.pred_grasps = self.pred_grasps[sorted_indicies]
        self.contact_pts = self.contact_pts[sorted_indicies]
        self.gripper_openings = self.gripper_openings[sorted_indicies]

        
    def request_grasp_callback(self,request: GraspReq.Request,response: GraspReq.Response):
        # Prep input
        # Expecting depth_image in meters
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(request.depth_image).copy().astype(np.float32)/1000
        self.rgb_image = self.cv_bridge.imgmsg_to_cv2(request.image,desired_encoding='rgb8').copy()
        self.segmentation_map = self.cv_bridge.imgmsg_to_cv2(request.segmentation_map).copy()
        intrinsics = request.camera_info.k
        self.k_intrinsics = np.array(intrinsics).reshape(3, 3)
        # Do inference
        self.run_network()
        # Visualize results 
        torch.cuda.empty_cache()
        # Prep response
        if(len(self.scores)>0):
            self.sort_results()
            response.success = True
            response.grasp_scores = self.scores.tolist()
            response.gripper_openings = self.gripper_openings.tolist()
            response.grasp_visualization = self.cv_bridge.cv2_to_imgmsg(self.grasp_viz,encoding='rgb8')
            print(response.grasp_visualization.encoding)
            for i in range(len(self.scores)):
                response.grasp_poses.append(pose_matrix_to_msg(self.pred_grasps[i,:,:]))
                response.contact_points.append(point_array_to_msg(self.contact_pts[i,:]))
        else:
            response.success = False
        return response

def main(args=None):
    rclpy.init(args=args)
    grasp_gen_node = Grasp_Generator()
    rclpy.spin(grasp_gen_node)

    grasp_gen_node.destroy_node()
    rclpy.shutdown()