import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_system_default
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 
from geometry_msgs.msg import Vector3Stamped
from custom_interfaces.msg import ImageMeta
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation


class ImageAquisition(Node):
    '''Node for aquiring the image from US probe and publishing it on the network with coresponding angle
    Extends Node'''
    def __init__(self):
        super().__init__('ImageAqusitionNode')

        # start CvBridge for converting between images and ros msgs
        self.bridge = CvBridge()

        # initialize transfrom listener to obtain the transformation between robot base and TCP
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # start publisher for US image
        self.img_pub = self.create_publisher(ImageMeta,'US_image',qos_profile_system_default)

        # timer for periodicaly captureing the image from US
        self.create_timer(1/46,self.get_image)

        # connecting to the US
        self.cap = cv2.VideoCapture(0)
        
        return

        

    def get_image(self):
        '''Callback function for the image capture timer
        Args : self
        Returns: None
        '''

        base_frame = 'lbr_link_0'
        TCP_frame = 'lbr_link_ee'

        # reading the image
        ret,frame = self.cap.read()
        # reading current ros time
        stamp = self.get_clock().now().to_msg()

        # if image could not be red skip
        if not ret:
            self.get_logger().warn('No picture')
            return
        

        # resizing the frame and croping out area with US picture
        frame = cv2.resize(frame, (1920, 1080))
        cv_image = frame[125:720, 645:1345]

        # transfering the image into ros msg format
        ros_image = self.bridge.cv2_to_imgmsg(cv_image)
        ros_image.header.stamp = stamp

        # try loop for reading the TCP transform, since node is started usualy before robot's controller at first there will be no data for transform and that will cause errors
        try:
            trans = self.tf_buffer.lookup_transform(TCP_frame,base_frame,rclpy.time.Time())

            # transforming obtainede quternion rotations of TCP to euler and extracting the rotation around the axis of the probe
            # KUKA uses zy'x'' notation of euler angles
            rot = Rotation.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
            angle = rot.as_euler('ZYX',degrees=False)[2]
        # when transformation could not be obtained angle is noted sa infinite    
        except Exception as e:
            #self.get_logger().warning('dogodila se graska : {}'.format(e))
            angle = float('inf')

        # creating an publishing the msg
        msg = ImageMeta()
        msg.image = ros_image
        msg.angle = angle
        self.img_pub.publish(msg)
        return

def main():
    '''main function, starts the node and spins it'''
    rclpy.init()
    node = ImageAquisition()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()

if __name__ == '__main__':
    main()