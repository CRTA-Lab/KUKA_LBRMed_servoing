import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default,qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped,Vector3Stamped
import time
#import cv2
import numpy as np

class PID():
    def __init__(self,Kp,Ki,Kd,T):
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        self.T = T
        self.integral = 0
        self.prev = 0
        return
    
    def update(self,error):
        self.integral += error*self.T
        diff = (error-self.prev)/self.T

        self.prev = error

        controll = self.kp*error + self.ki*self.integral + self.kd*diff
        return controll





class ServoingNode(Node):
    def __init__(self):
        time.sleep(1)
        super().__init__('LBRServoing')

        self.declare_parameter('mode','')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        self.xPID = PID(0.8,0,0,1/30)
        self.yPID = PID(0.8,0,0,1/30)
        self.zPID = PID(0.5,0,0,1/30)

        self.position_publisher = self.create_publisher(TwistStamped,"/lbr/servo_node/delta_twist_cmds",qos_profile_sensor_data)

        #self.create_subscription(Vector3Stamped,'phantom_position',self.postion_callback,qos_profile_system_default)
        self.create_subscription(Vector3Stamped,'phantom_position_filt',self.postion_filt_callback,qos_profile_system_default)

        self.prev = 0
        self.direction = 1


    def postion_callback(self,msg:Vector3Stamped):
        x = -msg.vector.x
        y = -msg.vector.y
        angle = msg.vector.z
        if np.sign(self.prev-angle):
            self.direction *= np.sign(self.prev-angle)
        pub_msg = TwistStamped()
        pub_msg.twist.linear.x = self.xPID.update(x)
        pub_msg.twist.linear.y = self.yPID.update(y)
        pub_msg.twist.linear.z = self.zPID.update(self.direction*angle)
        #pub_msg.twist.linear.z = -x/100 - y/100
        pub_msg.header.stamp = self.get_clock().now().to_msg()
        self.position_publisher.publish(pub_msg)
        self.prev = angle
        return

    def postion_filt_callback(self,msg:Vector3Stamped):
        x = -msg.vector.x
        y = -msg.vector.y
        angle = msg.vector.z
        if np.sign(self.prev-angle):
            self.direction *= np.sign(self.prev-angle)
        pub_msg = TwistStamped()
        pub_msg.twist.linear.x = self.xPID.update(x)
        pub_msg.twist.linear.y = self.yPID.update(y)
        pub_msg.twist.linear.z = self.zPID.update(self.direction*angle)
        #pub_msg.twist.linear.z = -x/100 - y/100
        pub_msg.header.stamp = self.get_clock().now().to_msg()
        self.position_publisher.publish(pub_msg)
        self.prev = angle
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