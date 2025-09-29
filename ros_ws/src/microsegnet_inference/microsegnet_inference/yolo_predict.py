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
from ultralytics import YOLO


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

        # timer for periodicaly captureing the image from US
        self.init_socket()
        self.create_timer(1/20,self.tcp_img_acq)

        self.force_in_range = False

        model_path = '/home/racunalo/KUKA_LBRMed_servoing/ros_ws/src/microsegnet_inference/resource/best.pt'
        self.model = YOLO(model_path)  # pretrained YOLO11n model

        
        return


    def init_socket(self):
        '''Initializes the socket connection for image reception.'''
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(('192.168.0.150', 9999))
        self.get_logger().info('Socket initialized.')
        return
    

    def tcp_img_acq(self):
        header = self.sock.recv(8)
        stamp = self.get_clock().now().to_msg()
        if not header:
            return
        height, width = struct.unpack('>II', header)

        # raw image data
        buffer_size = height * width
        data = b''
        while len(data) < buffer_size:
            packet = self.sock.recv(buffer_size - len(data))
            if not packet:
                break
            data += packet

        frame = np.frombuffer(data, dtype=np.uint8).reshape((height, width))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        cv2.imshow("US Scan", frame)
        
        base_frame = 'lbr_link_0'
        TCP_frame = 'lbr_link_ee'
        sweep_frame = 'init_pose'

        ros_image = self.bridge.cv2_to_imgmsg(frame)
        ros_image.header.stamp = stamp

        #try loop for reading the TCP transform, since node is started usualy before robot's controller at first there will be no data for transform and that will cause errors
        try:
            trans = self.tf_buffer.lookup_transform(sweep_frame,TCP_frame,rclpy.time.Time())

            # transforming obtained quternion rotations of TCP to euler and extracting the rotation around the axis of the probe
            # KUKA uses zy'x'' notation of euler angles
            rot = Rotation.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
            angle = rot.as_euler('ZYX',degrees=False)[2]
        except Exception as e:
            #self.get_logger().warning('dogodila se graska : {}'.format(e))
            try:
                trans = self.tf_buffer.lookup_transform(base_frame,TCP_frame,rclpy.time.Time())

                # transforming obtainede quternion rotations of TCP to euler and extracting the rotation around the axis of the probe
                # KUKA uses zy'x'' notation of euler angles
                rot = Rotation.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
                angle = rot.as_euler('ZYX',degrees=False)[2]
            # when transformation could not be obtained angle is noted sa infinite    
            except Exception as e:
                #self.get_logger().warning('dogodila se graska : {}'.format(e))
                angle = float('inf')

        # creating an publishing the msg
        msg = ImageMeta()
        msg.image = ros_image
        msg.angle = angle
        msg.force_in_range = self.force_in_range
        self.img_pub.publish(msg)

        results = self.model.predict(frame,stream=True,conf = 0.8,task='segment')

        #Process results generator
        for result in results:
            boxes = result.boxes  # Boxes object for bounding box outputs
            for box in boxes:
                # print(box.xyxy)  # x1, y1, x2, y2
                # print(box.xywh)  # x, y, w, h
                # print(box.conf)  # confidence
                # print(box.cls)  # class
                # print(box.prob)  # class probabilities (multiclass only)
                x1,y1,x2,y2 = map(int,box.xyxy[0])
                cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)
            #masks = result.masks  # Masks object for segmentation masks outputs
            # keypoints = result.keypoints  # Keypoints object for pose outputs
            # probs = result.probs  # Probs object for classification outputs
            # obb = result.obb  # Oriented boxes object for OBB outputs
            # result.show()  # display to screen
        cv2.imshow("yolo", frame)
        cv2.waitKey(1)
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