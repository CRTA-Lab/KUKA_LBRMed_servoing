#from email.mime import image
import struct
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_system_default
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 
from geometry_msgs.msg import Vector3Stamped, WrenchStamped
from sensor_msgs.msg import JointState
from custom_interfaces.msg import ImageMeta
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
import socket
 


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

        self.pub_sub_cbg = ReentrantCallbackGroup()

        # start publisher for US image
        self.img_pub = self.create_publisher(ImageMeta,'US_image',qos_profile_system_default,callback_group=self.pub_sub_cbg)

        #self.create_subscription(WrenchStamped,'/lbr/force_torque_broadcaster/wrench',self.ft_callback,qos_profile_system_default,callback_group=self.pub_sub_cbg)
        #TODO vratiti na staro
        self.create_subscription(WrenchStamped,'/ft_sensor_data',self.ft_callback,qos_profile_system_default,callback_group=self.pub_sub_cbg)

        # timer for periodicaly captureing the image from US
        self.init_socket()
        self.create_timer(1/20,self.tcp_img_acq)

        #TODO vratit na staro
        self.force_in_range = False

        

        # connecting to the US
        #self.cap = cv2.VideoCapture(0)
        
        return


    def init_socket(self):
        '''Initializes the socket connection for image reception.'''
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #self.sock.connect(('192.168.0.150', 9999))
        self.sock.connect(('192.168.0.168', 9999))
        self.get_logger().info('Socket initialized.')
        return
    

    def tcp_img_acq(self):
        self.sock.send(b'a')
        stamp = self.get_clock().now().to_msg()
        total_bytes = self.sock.recv(4)
        if not total_bytes:
            return
        total_bytes = struct.unpack('<I', total_bytes)[0]
        #self.get_logger().info(f'Total bytes to receive: {total_bytes}')
        header_bytes = self.sock.recv(40)
        if not header_bytes:
            return
        pixel_bytes = self.sock.recv(total_bytes - 40)
        while len(pixel_bytes) < (total_bytes - 40):
            #self.get_logger().info('Receiving image data... ({})'.format(len(pixel_bytes)/(total_bytes - 40)))
            more_bytes = self.sock.recv((total_bytes - 40) - len(pixel_bytes))
            if not more_bytes:
                break
            pixel_bytes += more_bytes
        if not pixel_bytes:
            return
        file_size = 14+total_bytes
        bmp_file_header = struct.pack('<2sIHHI', b'BM', file_size, 0, 0, 14+40)
        bmp_bytes = bmp_file_header + header_bytes + pixel_bytes
        frame = np.frombuffer(bmp_bytes, dtype=np.uint8)
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR_RGB)
        # header = self.sock.recv(8)
        # 
        # if not header:
        #     return
        # height, width = struct.unpack('>II', header)

        # # raw image data
        # buffer_size = height * width
        # data = b''
        # while len(data) < buffer_size:
        #     packet = self.sock.recv(buffer_size - len(data))
        #     if not packet:
        #         break
        #     data += packet

        # frame = np.frombuffer(data, dtype=np.uint8).reshape((height, width))
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        #self.get_logger().info(f'Image received: {frame.shape}',throttle_duration_sec=0.5)
        # cv2.line(frame, (0, frame.shape[0] // 6), (frame.shape[1],  frame.shape[0] // 6), (0, 0, 255), 1)
        # cv2.line(frame, (0, 5*frame.shape[0] // 6), (frame.shape[1],  5*frame.shape[0] // 6), (0, 0, 255), 1)
        # cv2.line(frame, (frame.shape[1] // 2, 0), (frame.shape[1] // 2, frame.shape[0]), (0, 0, 255), 1)
        # cv2.line(frame, (frame.shape[1] // 6, 0), (frame.shape[1] // 6, frame.shape[0]), (0, 0, 255), 1)
        # cv2.line(frame, (5*frame.shape[1] // 6, 0), (5*frame.shape[1] // 6, frame.shape[0]), (0, 0, 255), 1)
        # cv2.imshow("US Scan", frame)
        # cv2.waitKey(0)
        
        
        base_frame = 'lbr_link_0'
        TCP_frame = 'lbr_link_ee'
        sweep_frame = 'init_pose'

        ros_image = self.bridge.cv2_to_imgmsg(frame)
        ros_image.header.stamp = stamp

        msg = ImageMeta()

        #try loop for reading the TCP transform, since node is started usualy before robot's controller at first there will be no data for transform and that will cause errors
        try:
            trans = self.tf_buffer.lookup_transform(sweep_frame,TCP_frame,rclpy.time.Time())

            msg.x = trans.transform.translation.x
            msg.y = trans.transform.translation.y
            msg.z = trans.transform.translation.z

            # transforming obtained quternion rotations of TCP to euler and extracting the rotation around the axis of the probe
            # KUKA uses zy'x'' notation of euler angles
            rot = Rotation.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
            angle = rot.as_euler('ZYX',degrees=False)[2]
        except Exception as e:
            self.get_logger().warning('dogodila se graska : {}'.format(e))
            try:
                trans = self.tf_buffer.lookup_transform(base_frame,TCP_frame,rclpy.time.Time())

                # transforming obtainede quternion rotations of TCP to euler and extracting the rotation around the axis of the probe
                # KUKA uses zy'x'' notation of euler angles
                rot = Rotation.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
                angle = rot.as_euler('ZYX',degrees=False)[2]
            # when transformation could not be obtained angle is noted sa infinite    
            except Exception as e:
                self.get_logger().warning('dogodila se graska : {}'.format(e))
                angle = float('inf')

        # creating an publishing the msg
        msg.image = ros_image
        msg.angle = angle
        msg.force_in_range = self.force_in_range
        #msg.force_in_range = True
        self.img_pub.publish(msg)
        return

        #add lines for calibration
        cv2.line(frame, (0, frame.shape[0] // 6), (frame.shape[1],  frame.shape[0] // 6), (0, 0, 255), 1)
        cv2.line(frame, (0, 5*frame.shape[0] // 6), (frame.shape[1],  5*frame.shape[0] // 6), (0, 0, 255), 1)
        cv2.line(frame, (frame.shape[1] // 2, 0), (frame.shape[1] // 2, frame.shape[0]), (0, 0, 255), 1)
        cv2.line(frame, (frame.shape[1] // 6, 0), (frame.shape[1] // 6, frame.shape[0]), (0, 0, 255), 1)
        cv2.line(frame, (5*frame.shape[1] // 6, 0), (5*frame.shape[1] // 6, frame.shape[0]), (0, 0, 255), 1)
        cv2.imshow('Stylus with Intersection', frame)
        cv2.waitKey(1)
        return


    def ft_callback(self, msg: WrenchStamped):
        '''Callback for the force-torque sensor data.
        Args:        msg (WrenchStamped): message containing the force-torque data
        Returns:     None'''
        if 6.5 < msg.wrench.force.y < 7.5 and -1 < msg.wrench.force.z < 1:
            self.force_in_range = True
            return
        else:
            self.force_in_range = False
            return



    def get_image(self):
        '''Callback function for the image capture timer
        Args : self
        Returns: None
        '''

        base_frame = 'lbr_link_0'
        TCP_frame = 'lbr_link_ee'
        sweep_frame = 'init_pose'

        # reading the image
        ret,frame = self.cap.read()
        # reading current ros time
        stamp = self.get_clock().now().to_msg()

        # if image could not be red skip
        if not ret:
            #self.get_logger().warn('No picture')
            return
        

        # resizing the frame and croping out area with US picture
        frame = cv2.resize(frame, (1920, 1080))
        cv_image = frame[125:720, 645:1345]

        # transfering the image into ros msg format
        ros_image = self.bridge.cv2_to_imgmsg(cv_image)
        ros_image.header.stamp = stamp

        #try loop for reading the TCP transform, since node is started usualy before robot's controller at first there will be no data for transform and that will cause errors
        try:
            trans = self.tf_buffer.lookup_transform(sweep_frame,TCP_frame,rclpy.time.Time())

            # transforming obtained quternion rotations of TCP to euler and extracting the rotation around the axis of the probe
            # KUKA uses zy'x'' notation of euler angles
            rot = Rotation.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
            angle = rot.as_euler('ZYX',degrees=False)[2]
        except Exception as e:
            self.get_logger().warning('dogodila se graska : {}'.format(e))
            try:
                trans = self.tf_buffer.lookup_transform(base_frame,TCP_frame,rclpy.time.Time())

                # transforming obtainede quternion rotations of TCP to euler and extracting the rotation around the axis of the probe
                # KUKA uses zy'x'' notation of euler angles
                rot = Rotation.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
                angle = rot.as_euler('ZYX',degrees=False)[2]
            # when transformation could not be obtained angle is noted sa infinite    
            except Exception as e:
                self.get_logger().warning('dogodila se graska : {}'.format(e))
                angle = float('inf')

        # creating an publishing the msg
        msg = ImageMeta()
        msg.image = ros_image
        msg.angle = angle
        msg.force_in_range = self.force_in_range
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