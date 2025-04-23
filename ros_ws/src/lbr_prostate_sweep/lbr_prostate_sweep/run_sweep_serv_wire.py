import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default,qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros.buffer import Buffer
from tf2_ros import TransformListener, TransformBroadcaster
from custom_interfaces.msg import ImageMeta
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, WrenchStamped
from std_srvs.srv import Trigger
import cv2
from cv_bridge import CvBridge
import numpy as np
from scipy.spatial.transform import Rotation
import time
import math
import os


class SweepNode(Node):
    '''Node for performing the sweep with the robot'''
    def __init__(self):
        super().__init__('SweepNode')

        # start CvBridge for converting between images and ros msgs
        self.bridge = CvBridge()
        self.pub_sub_cbg = ReentrantCallbackGroup()
        self.serv_cbg = ReentrantCallbackGroup()
        self.time_cbg = ReentrantCallbackGroup()
        # create subscription on the US_image topic
        self.img_sub = self.create_subscription(ImageMeta,'/US_image',self.image_cb,qos_profile_system_default,callback_group=self.pub_sub_cbg)
        # create subscription on the wrench topic 
        self.ft_sub = self.create_subscription(WrenchStamped,'/lbr/force_torque_broadcaster/wrench',self.ft_callback,qos_profile_system_default,callback_group=self.pub_sub_cbg)

        # create publisher for moveit servo delta_twist_cmds topic
        self.twist_pub = self.create_publisher(TwistStamped,'/lbr/servo_node/delta_twist_cmds',qos_profile_sensor_data,callback_group=self.pub_sub_cbg)

        self.start_serv = self.create_service(Trigger,'/sweep/start',self.sweep_start,qos_profile=qos_profile_system_default,callback_group=self.serv_cbg)
        self.edge_serv = self.create_service(Trigger,'/sweep/edge_trigger',self.edge_triger,qos_profile=qos_profile_system_default,callback_group=self.serv_cbg)
        
        # initialize transfrom listener to obtain the transformation between robot base and TCP
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_timer = self.create_timer(1/100,self.broadcast_init_pose,callback_group=self.time_cbg)

        # initializeing the variables
        self.wrench = WrenchStamped()
        self.base_frame = 'lbr_link_0'
        self.TCP_frame = 'lbr_link_ee'
        self.start_flag = False
        self.end_flag = False
        self.init_pose = None
        self.back_flag = False
        self.changed_dir = False
        self.capture_flag = True
        
        self.img_path = '/home/racunalo1/KUKA_LBRMed_servoing/US_pics/sweep_P1_{}_{}'.format(time.localtime().tm_hour,time.localtime().tm_min)
        os.mkdir(self.img_path)
        os.mkdir(self.img_path + '/input_images')
        os.mkdir(self.img_path + '/output_images')
        #self.zcf = True # zero cross flag

        self.get_logger().info('Init done')

    def ft_callback(self,msg:WrenchStamped):
        self.wrench = msg
        return

    def sweep_start(self,request:Trigger.Request,response:Trigger.Response):
        '''Callback for the start service, sets class variable for starting the sweep to True'''
        self.start_flag = True
        self.get_logger().info('Sweep started')
        self.get_logger().info('Initial force = {}'.format(self.wrench))
        return response


    def edge_triger(self,request:Trigger.Request,response:Trigger.Response):
        '''Callback for the service when end of the prostate is detected'''
        self.back_flag = True
        self.get_logger().info('Changed directon')
        if self.changed_dir:
            self.end_flag = True
        else:
            self.changed_dir = True
        return response

    def initialize_pose(self):
        '''Method for initializing robots pose before the sweep. It is called when robot stats transmiting data
        Args : self
        Out: None
        '''
        self.init_pose = self.tf_buffer.lookup_transform(self.base_frame,self.TCP_frame,rclpy.time.Time())
        self.init_pose.child_frame_id = 'init_pose'
        #self.init_pose.header.frame_id = 'lbr_link_0'
        return


    def broadcast_init_pose(self):
        if self.init_pose != None:
            pose = self.init_pose
            pose.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(pose)
        return
    
    def find_blobs(self,image,angle):
        '''Method for finding the blobs in the image
        Args : self
                image - image to be processed
        Out : None
        '''
        # Set up the detector with default parameters.
        if not self.capture_flag:
            self.capture_flag = True
            return
        self.capture_flag = False
        cv2.imwrite(self.img_path + '/input_images/original_{}.jpg'.format(angle),image)
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 130
        params.maxThreshold = 255

        # Filter by Area
        params.filterByArea = True
        params.minArea = 5
        params.maxArea = 100

        # Filter by Circularity
        params.filterByCircularity = False

        # Filter by Convexity
        params.filterByConvexity = False

        # Filter by Inertia
        params.filterByInertia = False

        params.filterByColor = True
        params.blobColor = 255

        
        # Create a detector with the parameters
        detector = cv2.SimpleBlobDetector_create(params)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #thresh = cv2.adaptiveThreshold(image,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,5,2)
        thresh = cv2.threshold(gray,130,255,cv2.THRESH_BINARY)[1]
        cv2.imshow('orig',thresh)
        cv2.imwrite(self.img_path + '/output_images/slice_{}.jpg'.format(angle),thresh)

        # Detect blobs
        keypoints = detector.detect(image)

        blank = np.zeros(np.shape(image))

        for keypoint in keypoints:
            x = int(keypoint.pt[0])
            y = int(keypoint.pt[1])
            blank = cv2.circle(blank,(x,y),3,(0,0,255),-1)
        
        cv2.imshow('blobs',blank)
        cv2.waitKey(1)




    def image_cb(self,msg:ImageMeta):
        '''Method for performing the sweep. Method is called at for every new US image that gets sent over the network
        Args :  self
                msg - mesage receaved from subscription (ImageMeta)
        Out :   None
        '''
        # check if posiiton was initialized, if not don't start sweep
        
        if self.init_pose != None:
            if not self.start_flag:return
            if self.end_flag:
                self.get_logger().info('Sweep finished, stoping subscription')
                self.destroy_subscription(self.img_sub)
                return
            self.get_logger().info('Slice angle = {}'.format(msg.angle))
            self.get_logger().info('Force applied = {}'.format(math.sqrt(self.wrench.wrench.force.y**2 + self.wrench.wrench.force.z**2)))

            self.find_blobs(self.bridge.imgmsg_to_cv2(msg.image),msg.angle)

            # create and publish msg
            msg_out = TwistStamped()
            if self.back_flag:
                msg_out.twist.angular.x = -1.0
            else:
                msg_out.twist.angular.x = 1.0
            msg_out.header.stamp = self.get_clock().now().to_msg()
            self.twist_pub.publish(msg_out)

        # if position was not initialized, check if ther is valid position data and initialize the 
        elif msg.angle != float('inf'):
            self.get_logger().info('Inicijalizacija')
            self.initialize_pose()
            # msg_out = TwistStamped()
            # msg_out.twist.angular.x = 0.5
            # msg_out.header.stamp = self.get_clock().now().to_msg()
            # self.twist_pub.publish(msg_out)
            # self.back_flag = True
            self.get_logger().info('Iitialized with angle: {}'.format(self.init_pose))
        
        # displaying the US image
        cv_image = self.bridge.imgmsg_to_cv2(msg.image)
        cv2.imshow('image',cv_image)
        cv2.waitKey(1)




def main():
    '''main function, starts the node and spins it'''
    rclpy.init()

    node = SweepNode()

    while rclpy.ok():
        rclpy.spin_once(node,timeout_sec=0.1)
    node.destroy_node()
    return
   

if __name__ == '__main__':
    main()

