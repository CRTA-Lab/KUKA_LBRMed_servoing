import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default,qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
import numpy as np
from scipy.spatial.transform import Rotation
import time


class AimNode(Node):
    '''Node for performing the sweep with the robot'''
    def __init__(self):
        super().__init__('AimNode')

        # start CvBridge for converting between images and ros msgs
        self.pub_sub_cbg = ReentrantCallbackGroup()
        self.serv_cbg = ReentrantCallbackGroup()
        # create subscription on the US_image topic

        # create publisher for moveit servo delta_twist_cmds topic
        self.twist_pub = self.create_publisher(TwistStamped,'/lbr/servo_node/delta_twist_cmds',qos_profile_sensor_data,callback_group=self.pub_sub_cbg)

        self.target_serv = self.create_service(Trigger,'/sweep/target',self.sweep_start,qos_profile=qos_profile_system_default,callback_group=self.serv_cbg)
        #self.edge_serv = self.create_service(Trigger,'/sweep/edge_trigger',self.edge_triger,qos_profile=qos_profile_system_default,callback_group=self.serv_cbg)
        
        # initialize transfrom listener to obtain the transformation between robot base and TCP
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # initializeing the variables
        self.base_frame = 'lbr_link_0'
        self.TCP_frame = 'lbr_link_ee'

        self.get_logger().info('Init done')