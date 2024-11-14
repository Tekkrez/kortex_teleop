import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    
    def __init__(self):
        super().__init__('webcam_pub_node')
        self.publisher_ = self.create_publisher(Image,'image_raw',5)
        self.timer = self.create_timer(0.04,self.timer_callback)
        self.cap = cv2.VideoCapture(1)
        self.cv_bridge = CvBridge()
    def timer_callback(self):
        ret,frame = self.cap.read()
        frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        if ret == True:
            self.publisher_.publish(self.cv_bridge.cv2_to_imgmsg(frame,encoding='rgb8'))

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()