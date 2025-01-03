import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default,qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped,Vector3Stamped
import time
#import cv2
#import numpy as np



class ServoingNode(Node):
    def __init__(self):
        time.sleep(5)
        super().__init__('LBRServoing')

        self.declare_parameter('mode','')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        self.position_publisher = self.create_publisher(TwistStamped,"/lbr/servo_node/delta_twist_cmds",qos_profile_sensor_data)

        self.create_subscription(Vector3Stamped,'phantom_position',self.postion_callback,qos_profile_system_default)
        self.create_subscription(Vector3Stamped,'phantom_position_filt',self.postion_filt_callback,qos_profile_system_default)


    def postion_callback(self,msg:Vector3Stamped):
        pass

    def postion_filt_callback(self,msg:Vector3Stamped):
        x = msg.vector.x
        y = msg.vector.y
        pub_msg = TwistStamped()
        pub_msg.twist.linear.x = -(x-22.32)/30
        pub_msg.twist.linear.y = (y-15)/30
        #pub_msg.twist.linear.z = -x/100 - y/100
        pub_msg.header.stamp = self.get_clock().now().to_msg()
        self.position_publisher.publish(pub_msg)
        return
    

def main():
    rclpy.init()

    node = ServoingNode()


    while rclpy.ok():
        rclpy.spin_once(node,timeout_sec=0.1)
    node.destroy_node()
    return


if __name__ == '__main__':
    main()