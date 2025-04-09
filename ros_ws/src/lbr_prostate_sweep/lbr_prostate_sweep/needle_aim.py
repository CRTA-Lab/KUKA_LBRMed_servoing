import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default,qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
from custom_interfaces.srv import Target
import numpy as np
from scipy.spatial.transform import Rotation
import time


class AimNode(Node):
    '''Node for performing the wtargeting with the robot'''
    def __init__(self):
        super().__init__('AimNode')

        # start CvBridge for converting between images and ros msgs
        self.pub_sub_cbg = ReentrantCallbackGroup()
        self.serv_cbg = ReentrantCallbackGroup()
        # create subscription on the US_image topic

        # create publisher for moveit servo delta_twist_cmds topic
        self.twist_pub = self.create_publisher(TwistStamped,'/lbr/servo_node/delta_twist_cmds',qos_profile_sensor_data,callback_group=self.pub_sub_cbg)

        self.target_serv = self.create_service(Target,'/sweep/target',self.store_target,qos_profile=qos_profile_system_default,callback_group=self.serv_cbg)
        self.start_serv = self.create_service(Trigger,'/sweep/begin_biopsy',self.start_biopsy,qos_profile=qos_profile_system_default,callback_group=self.serv_cbg)
        self.next_target_serv = self.create_service(Trigger,'/sweep/next_target',self.next_target,qos_profile=qos_profile_system_default,callback_group=self.serv_cbg)
        
        # initialize transfrom listener to obtain the transformation between robot base and TCP
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(1/100, self.aim)

        # initializeing the variables
        self.base_frame = 'init_pose'
        self.TCP_frame = 'lbr_link_ee'
        self.targets = []
        self.target_no = 0
        self.start_flag = False

        self.get_logger().info('Init done')
        return
    
    def store_target(self,request:Target.Request,response:Target.Response):
        self.targets.apend((request.x,request.y,request.angle))
        response.ack = True
        return response
    
    def start_biopsy(self,request:Trigger.Request,response:Trigger.Response):
        response.success = True
        self.start_flag = True
        return response
    
    def next_target(self,request:Trigger.Request,response:Trigger.Response):
        response.success = True
        self.target_no += 1
        if self.target_no > len(self.targets):
            self.destroy_node()
        return response
    
    
    def aim(self):
        if not self.start_flag:return
        
        trans = self.tf_buffer.lookup_transform(self.base_frame,self.TCP_frame,rclpy.time.Time())

        # transforming obtainede quternion rotations of TCP to euler and extracting the rotation around the axis of the probe
        # KUKA uses zy'x'' notation of euler angles
        rot = Rotation.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
        angle = rot.as_euler('ZYX',degrees=False)[2]
        delta = self.targets[self.target_no]
        if abs(delta) > 0.01:
            msg = TwistStamped()
            msg.twist.angular.x = 2*delta
            msg.header.stamp = self.get_clock().now().to_msg()
            self.twist_pub.publish(msg)



def main():
    '''main function, starts the node and spins it'''
    rclpy.init()

    node = AimNode()

    while rclpy.ok():
        rclpy.spin_once(node,timeout_sec=0.1)
    node.destroy_node()
    return
   

if __name__ == '__main__':
    main()
