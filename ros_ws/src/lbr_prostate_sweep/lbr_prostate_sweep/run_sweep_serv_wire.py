import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default,qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros.buffer import Buffer
from tf2_ros import TransformListener, TransformBroadcaster, TransformStamped
from custom_interfaces.msg import ImageMeta
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, WrenchStamped, PoseStamped
from std_srvs.srv import Trigger
#from moveit.core import RobotModel
from moveit.core.planning_scene import PlanningScene
from moveit.planning import MoveItPy, PlanningComponent, PlanRequestParameters, PlanningSceneMonitor
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python import get_package_share_directory
import cv2
from cv_bridge import CvBridge
import numpy as np
from scipy.spatial.transform import Rotation
import time
import math
import os


def angle_conversion(euler):
    r = Rotation.from_euler('ZYX',euler,degrees=False)
    rot_angles = r.as_quat()
    print(rot_angles)
    return rot_angles

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
        self.phantom_frame = 'wire_frame'
        self.start_flag = False
        self.end_flag = False
        self.init_pose = None
        self.back_flag = False
        self.changed_dir = False
        self.capture_flag = True
        
        self.img_path = '/home/matija/KUKA_LBRMed_servoing/US_pics/sweep_P1_{}_{}'.format(time.localtime().tm_hour,time.localtime().tm_min)
        os.mkdir(self.img_path)
        os.mkdir(self.img_path + '/input_images')
        os.mkdir(self.img_path + '/output_images')
        self.zcf = True # zero cross flag

        #time.sleep(5)

        
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

        #time.sleep(1)


        # planing_scene = PlanningSceneMonitor(self,'robot_description','psm')
        # planing_scene.start_scene_monitor()
        # planing_scene.start_world_geometry_monitor()

        # self.lbr_arm = PlanningComponent('arm',planning_scene_monitor=planing_scene)

        # moveit_config_builder = MoveItConfigsBuilder('iiwa14_tool')
        # moveit_config_builder.moveit_cpp(file_path=get_package_share_directory("iiwa14_tool_moveit_config") + "/config/planing_pipeline_config.yaml")
        # moveit_config_builder.robot_description(os.path.join(get_package_share_directory('lbr_description'),'urdf/iiwa14_tool/iiwa14_tool.xacro'))
        # moveit_config_dict = moveit_config_builder.to_moveit_configs().to_dict()

        # self.lbr : MoveItPy = MoveItPy(node_name='moveit_py',config_dict = moveit_config_dict)
        # psm : PlanningSceneMonitor = self.lbr.get_planning_scene_monitor()
        # psm.start_scene_monitor('monitored_planning_scene')
        # self.lbr_arm : PlanningComponent = self.lbr.get_planning_component('arm')
        # self.get_logger().info('MoveItPy initialized')

        # planning_scene = PlanningScene(self.lbr.get_robot_model())

        # self.lbr_arm.set_start_state_to_current_state()
        # goal = self.tf_buffer.lookup_transform(self.base_frame,'target',rclpy.time.Time())
        # goal_pose = PoseStamped()
        # goal_pose.header.stamp = self.get_clock().now().to_msg()
        # goal_pose.header.frame_id = self.base_frame
        # goal_pose.pose.position.x = goal.transform.translation.x
        # goal_pose.pose.position.y = goal.transform.translation.y
        # goal_pose.pose.position.z = goal.transform.translation.z
        # goal_pose.pose.orientation.x = goal.transform.rotation.x
        # goal_pose.pose.orientation.y = goal.transform.rotation.y
        # goal_pose.pose.orientation.z = goal.transform.rotation.z
        # goal_pose.pose.orientation.w = goal.transform.rotation.w


        # self.lbr_arm.set_goal_state(pose_stamped_msg = goal_pose,pose_link = self.TCP_frame)
        # self.get_logger().info('Type : {}'.format(type(psm)))
        # try:
        #     plan = self.lbr_arm.plan(PlanRequestParameters(self.lbr,''))
        #     time.sleep(5)
        #     future = self.lbr.execute(plan.trajectory,[])
        # except Exception as e:
        #     self.get_logger().error('Error while planning : {}'.format(e))
        #     self.get_logger().info('Type : {}'.format(type(self.lbr_arm)))
            
        # rclpy.spin_until_future_complete(self,future)
        self.init_pose = self.tf_buffer.lookup_transform(self.base_frame,self.TCP_frame,rclpy.time.Time())
        self.init_pose.child_frame_id = 'init_pose'
        #self.init_pose.header.frame_id = 'lbr_link_0'
        return



    def broadcast_init_pose(self):
        pose = TransformStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'lbr_link_0'
        pose.child_frame_id = self.phantom_frame
        frame = [-193.62930448646964,555.7281724246884,76.66164793994926,2.8991537198756734,-1.4721502430548297,1.8866531711544678]
        pose.transform.translation.x = frame[0]/1000
        pose.transform.translation.y = frame[1]/1000
        pose.transform.translation.z = frame[2]/1000
        angle_quat = angle_conversion(frame[3:6])
        angle_quat = angle_conversion(frame[3:6])
        pose.transform.rotation.x = angle_quat[0]
        pose.transform.rotation.y = angle_quat[1]
        pose.transform.rotation.z = angle_quat[2]
        pose.transform.rotation.w = angle_quat[3]
        self.tf_broadcaster.sendTransform(pose)

        pose = TransformStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.phantom_frame
        pose.child_frame_id = 'target'
        pose.transform.translation.x = 0.05
        pose.transform.translation.y = -0.05
        pose.transform.translation.z = -0.05
        pose.transform.rotation.x = -1.0
        pose.transform.rotation.y = 0.0
        pose.transform.rotation.z = 0.0
        pose.transform.rotation.w = 0.0
        self.tf_broadcaster.sendTransform(pose)

        if self.init_pose != None:
            initial_pose = self.init_pose
            initial_pose.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(initial_pose)
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
        params.minThreshold = 45
        params.maxThreshold = 255

        # Filter by Area
        params.filterByArea = True
        params.maxArea = 120

        # Filter by Circularity
        params.filterByCircularity = False

        # Filter by Convexity
        params.filterByConvexity = False

        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.01

        params.filterByColor = True
        params.blobColor = 255

        
        # Create a detector with the parameters
        detector = cv2.SimpleBlobDetector_create(params)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #thresh = cv2.adaptiveThreshold(image,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,5,2)
        thresh = cv2.threshold(gray,45,255,cv2.THRESH_BINARY)[1]
        cv2.imshow('orig',thresh)
        

        # Detect blobs
        keypoints = detector.detect(image)

        blank = np.zeros(np.shape(gray))

        for keypoint in keypoints:
            x = int(keypoint.pt[0])
            y = int(keypoint.pt[1])
            blank = cv2.circle(blank,(x,y),3,255,-1)
        cv2.imwrite(self.img_path + '/output_images/slice_{:.5f}.jpg'.format(angle),blank)
        cv2.imshow('blobs',blank)
        cv2.waitKey(1)




    def image_cb(self,msg:ImageMeta):
        '''Method for performing the sweep. Method is called at for every new US image that gets sent over the network
        Args :  self
                msg - mesage receaved from subscription (ImageMeta)
        Out :   None
        '''
        # check if posiiton was initialized, if not don't start sweep
        #self.find_blobs(self.bridge.imgmsg_to_cv2(msg.image),msg.angle)
        if self.init_pose != None:
            if not self.start_flag:return
            if self.end_flag:
                self.get_logger().info('Sweep finished, stoping subscription')
                self.destroy_subscription(self.img_sub)
                return
            self.get_logger().info('Slice angle = {}'.format(msg.angle))
            self.get_logger().info('Force applied = {}'.format(math.sqrt(self.wrench.wrench.force.y**2 + self.wrench.wrench.force.z**2)))

            trans = self.tf_buffer.lookup_transform('wire_frame',self.TCP_frame,rclpy.time.Time())

            # transforming obtainede quternion rotations of TCP to euler and extracting the rotation around the axis of the probe
            # KUKA uses zy'x'' notation of euler angles
            rot = Rotation.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
            angle = rot.as_euler('ZYX',degrees=True)[2]
            if abs(msg.angle) > 0.26 and not self.zcf:
                self.edge_triger(Trigger.Request(),Trigger.Response())
                self.zcf = True
            elif abs(msg.angle) < 0.26 and self.zcf: self.zcf = False

            # create and publish msg
            msg_out = TwistStamped()
            if self.back_flag:
                self.find_blobs(self.bridge.imgmsg_to_cv2(msg.image),msg.angle)
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

