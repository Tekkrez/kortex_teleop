import rclpy
from rclpy.node import Node
from rclpy.task import Future
import rclpy.time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray
from teleop_interfaces.srv import GraspReq, ExecuteGrasp, GraspTrigger, VisualizeGrasp
from std_srvs.srv import SetBool
# from ultralytics import FastSAM
from ultralytics import SAM
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
        # Properties for gaze point
        self.left_gaze_point = None
        self.left_gaze_recieved = False
        self.right_gaze_recieved = False
        self.right_gaze_point = None
        self.center = [600,350]
        # Store grasp generation results
        self.grasp_requested_time = None
        self.grasp_poses: PoseArray = None
        self.grasp_scores = None
        self.grasp_contact_points = None
        self.gripper_openings = None
        # Grasp filter param
        self.grasps_generated = False
        self.max_distance_from_gaze = 0.06
        # Subscribers
        self.cam_info_sub = self.create_subscription(CameraInfo,"/head/right_camera/color/camera_info",self.set_camera_info,1)
        self.colour_sub = self.create_subscription(Image,"/head/right_camera/color/image_raw",self.colour_callback,3)
        self.depth_sub = self.create_subscription(Image,"/head/right_camera/aligned_depth_to_color/image_raw",self.depth_callback,3)
        
        # Publishers
        self.segmented_mask_pub = self.create_publisher(Image,"segmented_mask",3)
        self.segmented_image_pub = self.create_publisher(Image,"segmented_image",3)
        self.grasp_viz_pub = self.create_publisher(PoseArray,"grasp_poses_test",1)
        
        # Service server
        self.left_eye_server = self.create_service(srv_type=GraspTrigger,srv_name='left_trigger_grasp',callback=self.left_trigger_grasp_callback)
        self.right_eye_server = self.create_service(srv_type=GraspTrigger,srv_name='right_trigger_grasp',callback=self.right_trigger_grasp_callback)
        self.left_eye_choose_grasp_server = self.create_service(srv_type=GraspTrigger,srv_name='left_choose_grasp',callback=self.left_choose_grasp_callback)
        self.right_eye_choose_grasp_server = self.create_service(srv_type=GraspTrigger,srv_name='right_choose_grasp',callback=self.right_choose_grasp_callback)
        # Service client
        self.client = self.create_client(GraspReq,'request_grasp')
        self.execute_grasp_client = self.create_client(ExecuteGrasp,'execute_grasp')
        self.grasp_vis_client = self.create_client(VisualizeGrasp,'set_grasp_view')
        self.loading_grasp_ui_client = self.create_client(SetBool,'set_loading_ui')
        self.future : Future = None
        self.execute_grasp_future: Future = None
        self.grasp_viz_future: Future = None
        self.loading_ui_future: Future = None

        # Import network
        self.network = SAM("sam2.1_s.pt")

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

    def filter_grasps(self):
        k = np.array(self.camera_info_msg.k).reshape(3, 3)
        best_score = 0
        best_grasp_index = None
        x_gaze = (self.center[0]-k[0,2])/k[0,0]
        y_gaze = (self.center[1]-k[1,2])/k[1,1]
        center_gaze_unit_vector = np.array([x_gaze,y_gaze,1])
        center_gaze_unit_vector = center_gaze_unit_vector/np.linalg.norm(center_gaze_unit_vector)
        for i in range(len(self.grasp_poses)):
            pose = self.grasp_poses[i]
            # Check if the pose position is near the gaze point
            pose_unit_vector = np.array([pose.position.x,pose.position.y,pose.position.z])
            pose_unit_vector = pose_unit_vector/np.linalg.norm(pose_unit_vector)       
            # Find angle between the pose and the gaze point
            angle_between = np.arccos(np.dot(center_gaze_unit_vector,pose_unit_vector))
            self.get_logger().info(f"Angle between: {angle_between}")
            d = np.linalg.norm(np.array([pose.position.x,pose.position.y,pose.position.z]))*np.sin(angle_between)
            print(f"Distance from gaze: {d}")
            # TODO: Only need to check the first grasp that fits, already sorted by score
            if d<self.max_distance_from_gaze and self.grasp_scores[i]>best_score:
                best_score = self.grasp_scores[i]
                best_grasp_index = i
        return best_grasp_index

    def left_trigger_grasp_callback(self,request: GraspTrigger.Request,response: GraspTrigger.Response):
        # Callback for left gaze point
        self.right_gaze_recieved = True
        self.left_gaze_point = np.array([request.gaze_point.data[0],1-request.gaze_point.data[1]])

        # Run network if both gaze points are recieved
        if self.left_gaze_recieved and self.right_gaze_recieved:
            # Call service to show loading ui
            loading_grasp_ui_request = SetBool.Request()
            loading_grasp_ui_request.data = True
            self.loading_ui_future = self.loading_grasp_ui_client.call_async(loading_grasp_ui_request)
            print("Loading UI")
            # Reset flags
            self.left_gaze_recieved = False
            self.right_gaze_recieved = False
            self.grasp_requested_time = rclpy.time.Time()
            print("Grasp: requested now: ",rclpy.time.Time())
            # Average the gaze points
            center_gaze_point = (self.left_gaze_point+self.right_gaze_point)/2
            self.center = np.array([np.round(self.camera_info_msg.width*center_gaze_point[0]),np.round(self.camera_info_msg.height*center_gaze_point[1])],dtype=int).tolist()
            print(f"Center: {self.center}")
            self.grasp_requested = True
            
        response.success = True
        response.message = "Grasp requested"
        return response
    
    def right_trigger_grasp_callback(self,request: GraspTrigger.Request,response: GraspTrigger.Response):
        # Callback for right gaze point
        self.left_gaze_recieved = True
        self.right_gaze_point = np.array([request.gaze_point.data[0],1-request.gaze_point.data[1]])

        if self.left_gaze_recieved and self.right_gaze_recieved:
            # Call service to show loading ui
            loading_grasp_ui_request = SetBool.Request()
            loading_grasp_ui_request.data = True
            self.loading_ui_future = self.loading_grasp_ui_client.call_async(loading_grasp_ui_request)
            print("Loading UI")
            # Reset flags
            self.left_gaze_recieved = False
            self.right_gaze_recieved = False
            self.grasp_requested_time = rclpy.time.Time()
            # Print time when the request took place
            print("Grasp: requested now: ",rclpy.time.Time())
            # Average the gaze points
            center_gaze_point = (self.left_gaze_point+self.right_gaze_point)/2
            self.center = np.array([np.round(self.camera_info_msg.width*center_gaze_point[0]),np.round(self.camera_info_msg.height*center_gaze_point[1])],dtype=int).tolist()
            print(f"Center: {self.center}")
            self.grasp_requested = True
        
        response.success = True
        response.message = "Grasp requested"
        return response
    
    def left_choose_grasp_callback(self,request: GraspTrigger.Request,response: GraspTrigger.Response):
        self.left_gaze_recieved = True
        self.left_gaze_point = np.array([request.gaze_point.data[0],1-request.gaze_point.data[1]])

        if self.left_gaze_recieved and self.right_gaze_recieved and self.grasps_generated:
            self.left_gaze_recieved = False
            self.right_gaze_recieved = False
            # Average the gaze points
            center_gaze_point = (self.left_gaze_point+self.right_gaze_point)/2
            self.center = np.array([np.round(self.camera_info_msg.width*center_gaze_point[0]),np.round(self.camera_info_msg.height*center_gaze_point[1])],dtype=int).tolist()
            print(f"Center: {self.center}")
            best_fit_grasp_index = self.filter_grasps()
            # Execute grasp based on gaze,. If there are no gazes at target location, execute grasp with the highest confidence score
            if best_fit_grasp_index is not None:
                self.execute_grasp(best_fit_grasp_index)
            else:
                self.execute_grasp(0)

        response.success = True
        response.message = "Grasp chosen"
        return response
    
    def right_choose_grasp_callback(self,request: GraspTrigger.Request,response: GraspTrigger.Response):
        self.right_gaze_recieved = True
        self.right_gaze_point = np.array([request.gaze_point.data[0],1-request.gaze_point.data[1]])

        if self.left_gaze_recieved and self.right_gaze_recieved and self.grasps_generated:
            self.left_gaze_recieved = False
            self.right_gaze_recieved = False
            # Average the gaze points
            center_gaze_point = (self.left_gaze_point+self.right_gaze_point)/2
            self.center = np.array([np.round(self.camera_info_msg.width*center_gaze_point[0]),np.round(self.camera_info_msg.height*center_gaze_point[1])],dtype=int).tolist()
            print(f"Center: {self.center}")
            best_fit_grasp_index = self.filter_grasps()
            # Execute grasp based on gaze,. If there are no gazes at target location, execute grasp with the highest confidence score
            if best_fit_grasp_index is not None:
                self.execute_grasp(best_fit_grasp_index)
            else:
                self.execute_grasp(0)
        
        response.success = True
        response.message = "Grasp chosen"
        return response

    def run_network(self):
        # Request grasp
        if self.grasp_requested:
            # Reset flag
            self.grasp_requested = False
            self.grasps_generated = False
            # Segment image based on gaze point
            results = self.network(self.image,points = [self.center],conf=0.2,verbose=False)
            result: Results =  results[0]
            masks: Masks = result.masks
            collapsed_mask = torch.any(masks.data,dim=0).type(torch.uint8)*1
            segmented_mask = cv2.resize(collapsed_mask.cpu().numpy(),dsize=(result.orig_shape[1],result.orig_shape[0]))
            masked_img = np.where(segmented_mask[...,None],np.array([0,0,255], dtype='uint8'),self.image)
            # Visualize and send the segmented image
            segmented_image = cv2.addWeighted(self.image,0.7,masked_img,0.3,0)
            segmented_image = cv2.circle(segmented_image,center=tuple(self.center),radius=10,color=(0,255,0),thickness=2)
            segmented_image = cv2.cvtColor(segmented_image,cv2.COLOR_BGR2RGB)
            cv2.imwrite("segmented_image.jpg",segmented_image)
            segmented_image_msg = self.cv_bridge.cv2_to_imgmsg(segmented_image,'rgb8')
            segmented_image_msg.header.stamp = self.rgb_framestamp
            segmented_image_msg.header.frame_id = self.rgb_frameid
            self.segmented_image_pub.publish(segmented_image_msg)
            # Send the segmented mask
            segmented_mask_msg = self.cv_bridge.cv2_to_imgmsg(segmented_mask)
            segmented_mask_msg.header.stamp = self.rgb_framestamp
            segmented_mask_msg.header.frame_id = self.rgb_frameid
            segmented_mask_msg.encoding = 'mono8'
            self.segmented_mask_pub.publish(segmented_mask_msg)
            # Might not be needed
            torch.cuda.empty_cache()

            # Request Grasp
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
            
    def execute_grasp_response(self,future: Future):
        response = future.result()
        if response.success:
            print("GRASP EXECUTED")

    def grasp_viz_response(self,future: Future):
        response = future.result()
        if response.success:
            print("GRASP VISUALIZED")

    def process_response(self, future: Future):
        response = future.result()
        if response.success:
            # pose_array_message = PoseArray()
            # pose_array_message.poses = response.grasp_poses
            # pose_array_message.header.frame_id = "head_camera_colour_frame"
            # pose_array_message.header.stamp = rclpy.time.Time().to_msg()
            # self.grasp_viz_pub.publish(pose_array_message)

            # Store the results from execulte grasp
            self.grasps_generated = True
            self.grasp_poses = response.grasp_poses
            self.grasp_scores = response.grasp_scores
            self.grasp_contact_points = response.contact_points
            # self.gripper_openings = response.gripper_openings
            # execute_grasp_request = ExecuteGrasp.Request()
            # while not self.client.wait_for_service(timeout_sec=1.0) and rclpy.ok():
            #     self.get_logger().info(f'{self.client.srv_name} service not available, waiting...')
            
            # Visualize grasp
            set_grasp_view_request = VisualizeGrasp.Request()
            set_grasp_view_request.generated_grasps = True
            set_grasp_view_request.grasp_visualization = response.grasp_visualization
            # print time to process resonse
            print("Time after response: ",rclpy.time.Time())
            print(f"Time to process response: {rclpy.time.Time()-self.grasp_requested_time}")
            self.grasp_viz_future = self.grasp_vis_client.call_async(set_grasp_view_request)
            self.grasp_viz_future.add_done_callback(self.grasp_viz_response)
            ## Create request
            # execute_grasp_request.grasp_pose = response.grasp_poses[0]
            # execute_grasp_request.grasp_score = response.grasp_scores[0]
            # execute_grasp_request.contact_point = response.contact_points[0]
            # execute_grasp_request.gripper_opening = response.gripper_openings[0]
        else:
            set_grasp_view_request = VisualizeGrasp.Request()
            set_grasp_view_request.generated_grasps = False
            self.grasp_viz_future = self.grasp_vis_client.call_async(set_grasp_view_request)
            self.grasp_viz_future.add_done_callback(self.grasp_viz_response)
            print("NO GRASPS GENERATED")

    def execute_grasp(self,grasp_index):
        execute_grasp_request = ExecuteGrasp.Request()
        while not self.client.wait_for_service(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().info(f'{self.client.srv_name} service not available, waiting...')
        # Create request
        execute_grasp_request.grasp_pose = self.grasp_poses[grasp_index]
        execute_grasp_request.grasp_score = self.grasp_scores[grasp_index]
        execute_grasp_request.contact_point = self.grasp_contact_points[grasp_index]
        # execute_grasp_request.gripper_opening = self.gripper_openings[grasp_index]
        # Send request to execute grasp
        self.execute_grasp_future = self.execute_grasp_client.call_async(execute_grasp_request)
        self.execute_grasp_future.add_done_callback(self.execute_grasp_response)


def main(args=None):
    rclpy.init(args=args)
    grasp_requester_node = grasp_requester()
    rclpy.spin(grasp_requester_node)

    grasp_requester_node.destroy_node()
    rclpy.shutdown()