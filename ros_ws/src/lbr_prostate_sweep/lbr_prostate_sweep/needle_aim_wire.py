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
from sksurgerynditracker.nditracker import NDITracker
import ndicapy

class AimNode(Node):
    '''Node for performing the targeting with the robot'''
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

        self.tracker = self.camera_setup()

        self.targets_log = open('/home/racunalo/KUKA_LBRMed_servoing/US_pics/targets/targets.txt','w')

        # initializeing the variables
        self.sweep_frame = 'init_pose'
        self.base_frame = 'wire_frame'
        self.TCP_frame = 'lbr_link_ee'
        self.targets = []
        self.target_no = 0
        self.start_flag = False
        self.angle_flag = False
        self.position_flag = False
        self.needle_flag = False
        self.last_img = None
        self.img_save_flag = False
        self.goal_flag = False
        self.dist_prev = 10
        self.dir = 1

        self.probe_pose = 0
        

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
        probe_pose = self.tf_buffer.lookup_transform(self.base_frame,self.sweep_frame,rclpy.time.Time())
        probe_rot = Rotation.from_quat([probe_pose.transform.rotation.x, probe_pose.transform.rotation.y, probe_pose.transform.rotation.z, probe_pose.transform.rotation.w])
        probe_rot = probe_rot.as_matrix()
        self.probe_pose = np.eye(4)
        self.probe_pose[:3,:3] = probe_rot
        self.probe_pose[:3,3] = np.array([probe_pose.transform.translation.x*1000,
                                           probe_pose.transform.translation.y*1000,
                                           probe_pose.transform.translation.z*1000])
        self.get_logger().info('Probe pose: {}'.format(self.probe_pose))
        self.targets_log.writelines('Probe pose: {}\n'.format(self.probe_pose))
        self.start_flag = True
        self.get_logger().info('Rupa br : {}\tDubina : {} mm'.format(self.targets[self.target_no][1]//5+1,(60-self.targets[self.target_no][0])+82))
        return response
    
    def next_target(self,request:Trigger.Request,response:Trigger.Response):
        response.success = True
        if not self.needle_flag:
            self.needle_flag = True
            port_handles, timestamps, framenumbers, tracking, quality = self.tracker.get_frame()
            self.get_logger().info('port_handles: {}'.format(port_handles))
            needle_position = self.calculate_distance(tracking)
            target_position = self.calculate_target()
            self.get_logger().info('Target {}: target - {} || needle - {}\n'.format(self.target_no+1,self.targets[self.target_no],needle_position))
            self.targets_log.writelines('Target {}: target - {} || needle - {}\n'.format(self.target_no+1,self.targets[self.target_no],needle_position))
        else:
            self.needle_flag = False
            self.target_no += 1
            if self.target_no >= len(self.targets):
                self.get_logger().info('Targeting finished, destroying node')
                self.targets_log.close()
                
                self.destroy_node()
                return response
            self.position_flag = True
            self.get_logger().info('Rupa br : {}\tDubina : {}'.format(self.targets[self.target_no][1]//5+1,(60-self.targets[self.target_no][0])+82))
            
        return response
    
    def save_img(self,angle):
        if not self.img_save_flag:
            cv2.imwrite('/home/racunalo/KUKA_LBRMed_servoing/US_pics/targets/target_{}_{:.4f}.jpg'.format(self.target_no+1,angle),self.last_img)
            self.img_save_flag = True
            self.get_logger().info('Saved image for target {} at angle {}'.format(self.target_no+1,angle))
        return
    
    
    def camera_setup(self):
        '''
        Configures the connection to the camera and initiation of the tracking

        Args : none

        Output : NDITracker - camera comunication object
        '''
        SETTINGS = {
            "tracker type": "vega",
            "ip address": "192.168.0.120",
            "romfiles" : ["/home/racunalo/KUKA_LBRMed_servoing/markeri/8700339_wire_phantom.rom",
                        "/home/racunalo/KUKA_LBRMed_servoing/markeri/FEM0001_biopsy_needle.rom"],
            
            }
        TRACKER = NDITracker(SETTINGS)

        ndicapy.ndiCommand(TRACKER._device,'SET Param.Tracking.Track Frequency=2')
        

        TRACKER.start_tracking()
        return TRACKER
    
    def calculate_distance(self,transforms:list):
        """
        Calculates the transformation between phantom and needle markers from their transforms in coordinate frame of the camera

        Args :
            handles - list of the marker labels
            transforms - list of transformation matrices of each marker
        Output:
            translation - translation vector of the needle marker w/respect to phantom marker (ndarray[1,3])
        """
        T_phantom = np.array(transforms[0])
        T_needle = np.array(transforms[1])

        if np.isnan(T_phantom).any() or np.isnan(T_needle).any():
            return np.nan, np.nan
        T_needle_local = np.matmul(np.linalg.inv(T_phantom),T_needle)

        return T_needle_local[:3,-1]
    
    def calculate_target(self):
        rot = Rotation.from_euler('x',self.targets[self.target_no][2],degrees=True)
        rot_mat = np.zeros((4,4))
        rot_mat[:3,:3] = rot.as_matrix()
        rot_mat[3,3] = 1
        target_position = np.matmul(self.probe_pose ,rot_mat)
        trans_mat = np.eye(4)
        trans_mat[:3,3] = np.array([60-self.targets[self.target_no][0],self.targets[self.target_no][1]+9,0])
        target_position = np.matmul(target_position,trans_mat)
        return target_position[:3,3]



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
            self.save_img(self.targets[self.target_no][2])


        if not self.angle_flag:
            msg = TwistStamped()
            ctrl = 3*delta
            if ctrl > 1:
                ctrl = 1.0
            elif ctrl <-1:
                ctrl = -1.0
            msg.twist.angular.x = ctrl
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.TCP_frame
            self.twist_pub.publish(msg)
        else:
            trans = self.tf_buffer.lookup_transform(self.sweep_frame,self.TCP_frame,rclpy.time.Time())
            dist = math.sqrt((trans.transform.translation.y**2 + trans.transform.translation.z**2))*1000
            dist = abs(trans.transform.translation.y)*1000
            
            if not self.position_flag:
                delta_p = 5-(self.targets[self.target_no][1]%5) - dist
            else:
                self.get_logger().info('Dist: {}   delta_prev: {}    dir: {}'.format(dist,self.dist_prev,self.dir))
                if dist > self.dist_prev+0.05:
                    self.dir*=-1
                delta_p = -dist/2*self.dir
                self.dist_prev = dist
                if abs(delta_p) < 0.2:
                    self.get_logger().info('Returned to init position')
                    self.angle_flag = False
                    self.img_save_flag = False
                    self.position_flag = False
                    self.goal_flag = False
                    self.dist_prev = 10
                    self.dir = 1
            #self.get_logger().info('Current position: {}\tTarget position: {}\tDelta: {}'.format(dist,5-(self.targets[self.target_no][1]%5),delta_p))
            if abs(delta_p) > 0.1:
                msg = TwistStamped()
                ctrl = 0.015*delta_p
                if ctrl > 1:
                    ctrl = 1.0
                elif ctrl <-1:
                    ctrl = -1.0
                msg.twist.linear.y = ctrl
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.TCP_frame
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
