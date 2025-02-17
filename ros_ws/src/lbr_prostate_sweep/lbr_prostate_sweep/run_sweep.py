import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default,qos_profile_sensor_data
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 
from custom_interfaces.msg import ImageMeta
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
import cv2
from cv_bridge import CvBridge
import numpy as np
from scipy.spatial.transform import Rotation
import time



class SweepNode(Node):
    def __init__(self):
        super().__init__('SweepNode')

        self.bridge = CvBridge()
        
        self.create_subscription(ImageMeta,'/US_image',self.image_cb,qos_profile_system_default)
        self.twist_pub = self.create_publisher(TwistStamped,'/lbr/servo_node/delta_twist_cmds',qos_profile_sensor_data)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame = 'lbr_link_0'
        self.TCP_frame = 'lbr_link_ee'
        self.init_pose = None
        self.back_flag = False
        self.get_logger().info('Init done')

    def initialize_pose(self):
        pose = self.tf_buffer.lookup_transform(self.TCP_frame,self.base_frame,rclpy.time.Time())
        self.init_pose = Rotation.from_quat([pose.transform.rotation.x, pose.transform.rotation.y, pose.transform.rotation.z, pose.transform.rotation.w]).as_euler('ZYX',degrees=True)[2]
        #self.get_logger().info(self.init_pose)
        return

    def image_cb(self,msg:ImageMeta):
        #self.get_logger().info('Ulazi')            
        if self.init_pose != None:
            pose = self.tf_buffer.lookup_transform(self.TCP_frame,self.base_frame,rclpy.time.Time())
            pose = Rotation.from_quat([pose.transform.rotation.x, pose.transform.rotation.y, pose.transform.rotation.z, pose.transform.rotation.w]).as_euler('ZYX',degrees=True)[2]
            delta = pose - self.init_pose
            if delta > 180:
                delta -= 360
            elif delta < -180: 
                delta +=360
            
            if delta > 35:
                self.back_flag = True
            elif delta < -35:
                self.back_flag = False

            msg_out = TwistStamped()
            if self.back_flag:
                msg_out.twist.angular.x = 1.0
            else:
                msg_out.twist.angular.x = -1.0
            msg_out.header.stamp = self.get_clock().now().to_msg()
            self.twist_pub.publish(msg_out)


        elif msg.angle != float('inf'):
            self.get_logger().info('Inicijalizacija')
            self.initialize_pose()
            msg_out = TwistStamped()
            msg_out.twist.angular.x = 0.5
            msg_out.header.stamp = self.get_clock().now().to_msg()
            self.twist_pub.publish(msg_out)
            self.back_flag = True
            self.get_logger().info('Iitialized with angle: {}'.format(self.init_pose))
        cv_image = self.bridge.imgmsg_to_cv2(msg.image)
        cv2.imshow('image',cv_image)
        cv2.waitKey(1)




def main():
    rclpy.init()

    node = SweepNode()

    while rclpy.ok():
        rclpy.spin_once(node,timeout_sec=0.1)
    node.destroy_node()
    return
   

if __name__ == '__main__':
    main()

