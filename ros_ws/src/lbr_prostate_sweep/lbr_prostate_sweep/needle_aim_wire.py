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
from custom_interfaces.msg import ImageMeta
import numpy as np
import math
from scipy.spatial.transform import Rotation
import time
import cv2
from cv_bridge import CvBridge
class AimNode(Node):
    '''Node for performing the wtargeting with the robot'''
    def __init__(self):
        super().__init__('AimNode')

        self.bridge = CvBridge()

        self.pub_sub_cbg = ReentrantCallbackGroup()
        self.serv_cbg = ReentrantCallbackGroup()
        # create subscription on the US_image topic

        # create publisher for moveit servo delta_twist_cmds topic
        self.twist_pub = self.create_publisher(TwistStamped,'/lbr/servo_node/delta_twist_cmds',qos_profile_sensor_data,callback_group=self.pub_sub_cbg)
        self.create_subscription(ImageMeta,'/US_image',self.image_cb,qos_profile_system_default,callback_group=self.pub_sub_cbg)

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
        self.angle_flag = False
        self.position_flag = False
        self.last_img = None
        self.img_save_flag = False
        self.goal_flag = False
        

        self.get_logger().info('Init done')
        return

    def image_cb(self,msg:ImageMeta):
        self.last_img =self.bridge.imgmsg_to_cv2(msg.image)
        return

    def store_target(self,request:Target.Request,response:Target.Response):
        self.targets.append((request.x/12,request.y/12,request.angle))
        self.get_logger().info('Saved point : {}'.format(self.targets[-1]))
        response.ack = True
        return response
    
    def start_biopsy(self,request:Trigger.Request,response:Trigger.Response):
        response.success = True
        self.start_flag = True
        self.get_logger().info('Rupa br : {}'.format(self.targets[self.target_no][1]//5+1))
        return response
    
    def next_target(self,request:Trigger.Request,response:Trigger.Response):
        response.success = True
        self.target_no += 1
        self.position_flag = True
        self.get_logger().info('Rupa br : {}'.format(self.targets[self.target_no][1]//5+1))
        if self.target_no > len(self.targets):
            self.destroy_node()
        return response
    
    def save_img(self):
        if not self.img_save_flag:
            cv2.imwrite('/home/racunalo1/KUKA_LBRMed_servoing/US_pics/targets/target_{}.jpg'.format(self.target_no+1),self.last_img)
            self.img_save_flag = True
        return
    
    
    def aim(self):
        if not self.start_flag:return
        
        trans = self.tf_buffer.lookup_transform(self.base_frame,self.TCP_frame,rclpy.time.Time())

        # transforming obtainede quternion rotations of TCP to euler and extracting the rotation around the axis of the probe
        # KUKA uses zy'x'' notation of euler angles
        rot = Rotation.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
        angle = rot.as_euler('ZYX',degrees=False)[2]
        delta = self.targets[self.target_no][2] - angle
        #self.get_logger().info('Current angle: {}\tTarget angle: {}\tDelta: {}'.format(angle,self.targets[self.target_no][2],delta))
        if abs(delta) < 0.001:
            self.angle_flag = True
            self.save_img()

        if not self.angle_flag:
            msg = TwistStamped()
            ctrl = 5*delta
            if ctrl > 1:
                ctrl = float(1)
            elif ctrl <-1:
                ctrl = float(-1)
            msg.twist.angular.x = ctrl
            msg.header.stamp = self.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
        else: 
            dist = math.sqrt((trans.transform.translation.y**2 + trans.transform.translation.z**2))*1000
            
            if not self.position_flag:
                delta_p = 5-(self.targets[self.target_no][1]%5) - dist
            else:
                delta_p = -dist
                if abs(delta_p) < 0.1:
                    self.angle_flag = False
                    self.img_save_flag = False
                    self.position_flag = False
                    self.goal_flag = False
            #self.get_logger().info('Current position: {}\tTarget position: {}\tDelta: {}'.format(dist,5-(self.targets[self.target_no][1]%5),delta_p))
            if abs(delta_p) > 0.1:
                msg = TwistStamped()
                ctrl = 0.15*delta_p
                if ctrl > 1:
                    ctrl = float(1)
                elif ctrl <-1:
                    ctrl = float(-1)
                msg.twist.linear.y = ctrl
                msg.header.stamp = self.get_clock().now().to_msg()
                self.twist_pub.publish(msg)
            elif not self.goal_flag:
                self.get_logger().info('target reached')
                self.goal_flag = True



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
