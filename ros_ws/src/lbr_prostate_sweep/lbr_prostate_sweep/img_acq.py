import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import Vector3Stamped
from custom_interfaces.msg import ImageMeta
from cv_bridge import CvBridge

class ImageAquisition(Node):
    def __init__(self):
        super().__init__('ImageAqusitionNode')

        self.bridge = CvBridge()

        self.img_pub = self.create_publisher(ImageMeta,'US_iamge',qos_profile_system_default)

        self.create_timer(1/46,self.get_image)

        # connecting to the camera
        self.cap = cv2.VideoCapture(0)
        return

        

    def get_image(self):
        ret,frame = self.cap.read()
        stamp = self.get_clock().now().to_msg()
        if not ret:
            self.position_publisher.publish(Vector3Stamped())
            self.get_logger().warn('No picture')
            return
        
        # resizing the frame and croping out are with US picture
        frame = cv2.resize(frame, (1920, 1080))
        cv_image = frame[125:720, 645:1345]
        ros_image = self.bridge.cv2_to_imgmsg(cv_image)
        ros_image.header.stamp = stamp
        msg = ImageMeta()
        msg.image = ros_image
        msg.angle = np.pi
        self.img_pub.publish(msg)
        return

def main():
    rclpy.init()
    node = ImageAquisition()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()

if __name__ == '__main__':
    main()