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
from sensor_msgs.msg import JointState, Image
from custom_interfaces.msg import ImageMeta
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
import socket
from ultralytics import YOLO


def filter_main_object(mask, min_area_ratio=0.5, distance_threshold=500):
    """
    Removes small/distant blobs from a YOLO segmentation mask.
    By ChatGPT
    """
    # Ensure mask is uint8 binary
    mask = (mask > 0).astype(np.uint8) * 255

    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    print(mask.shape)
    print(labels.shape)

    if num_labels <= 1:
        return mask

    # Find largest blob (main object)
    largest_idx = np.argmax(stats[1:, cv2.CC_STAT_AREA]) + 1
    largest_centroid = centroids[largest_idx]
    largest_area = stats[largest_idx, cv2.CC_STAT_AREA]

    filtered_mask = np.zeros_like(mask)

    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        centroid = centroids[i]
        distance = np.linalg.norm(centroid - largest_centroid)

        if area > largest_area * min_area_ratio and distance < distance_threshold:
            filtered_mask[labels == i] = 255

    return filtered_mask

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
        #self.img_pub = self.create_publisher(ImageMeta,'US_image',qos_profile_system_default,callback_group=self.pub_sub_cbg)

        self.img_pub = self.create_publisher(Image,'conv_img',qos_profile_system_default,callback_group=self.pub_sub_cbg)
        self.position_pub = self.create_publisher(Vector3Stamped,'conv_element_position',qos_profile_system_default,callback_group=self.pub_sub_cbg)

        # timer for periodicaly captureing the image from US
        self.init_socket()
        self.create_timer(1/20,self.tcp_img_acq)

        self.force_in_range = False

        model_path = '/home/racunalo/KUKA_LBRMed_servoing/ros_ws/src/microsegnet_inference/resource/Urethra_model_v2.pt'
        self.model = YOLO(model_path)  # pretrained YOLO11n model

        self.key = 0
        self.pic_no = 1
        
        return


    def init_socket(self):
        '''Initializes the socket connection for image reception.'''
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
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
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR_BGR)
        cv2.imshow("US Scan", frame)
        #cv2.waitKey(1)
        
        #TODO vratit ovaj dio da se slika salje na ros topic
        # base_frame = 'lbr_link_0'
        # TCP_frame = 'lbr_link_ee'
        # sweep_frame = 'init_pose'

        ros_image = self.bridge.cv2_to_imgmsg(frame)
        ros_image.header.stamp = stamp

        # #try loop for reading the TCP transform, since node is started usualy before robot's controller at first there will be no data for transform and that will cause errors
        # try:
        #     trans = self.tf_buffer.lookup_transform(sweep_frame,TCP_frame,rclpy.time.Time())

        #     # transforming obtained quternion rotations of TCP to euler and extracting the rotation around the axis of the probe
        #     # KUKA uses zy'x'' notation of euler angles
        #     rot = Rotation.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
        #     angle = rot.as_euler('ZYX',degrees=False)[2]
        # except Exception as e:
        #     #self.get_logger().warning('dogodila se graska : {}'.format(e))
        #     try:
        #         trans = self.tf_buffer.lookup_transform(base_frame,TCP_frame,rclpy.time.Time())

        #         # transforming obtainede quternion rotations of TCP to euler and extracting the rotation around the axis of the probe
        #         # KUKA uses zy'x'' notation of euler angles
        #         rot = Rotation.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
        #         angle = rot.as_euler('ZYX',degrees=False)[2]
        #     # when transformation could not be obtained angle is noted sa infinite    
        #     except Exception as e:
        #         #self.get_logger().warning('dogodila se graska : {}'.format(e))
        #         angle = float('inf')

        # # creating an publishing the msg
        #frame = cv2.resize(frame, (640, 640), interpolation=cv2.INTER_AREA)
        #frame = cv2.blur(frame, (3,3))
        if self.key == ord('s'):
            cv2.imwrite('/home/racunalo/KUKA_LBRMed_servoing/US_pics/temp/pic_{}.jpg'.format(self.pic_no),frame)
            self.get_logger().info('Picture saved: pic_{}.jpg'.format(self.pic_no))
            self.pic_no +=1

        cv2.blur(frame, (3,3))
        img_arr = np.array(frame)

        results = self.model.predict(frame,stream=True,conf = 0.5,task='segment', iou=0.1, verbose=True,show = False)
        #Process results generator
        for result in results:
            boxes = result.boxes  # Boxes object for bounding box outputs
            # if boxes is None:
            #     self.get_logger().info('No boxes detected')
            # else:
            #     for box in boxes:
            #         # print(box.xyxy)  # x1, y1, x2, y2
            #         # print(box.xywh)  # x, y, w, h
            #         # print(box.conf)  # confidence
            #         # print(box.cls)  # class
            #         # print(box.prob)  # class probabilities (multiclass only)

            #         # x1,y1,x2,y2,x3,y3,x4,y4 = map(int,box.xyxyxyxy[0])
            #         # cv2.drawContours(frame, [np.array([[x1,y1],[x2,y2],[x3,y3],[x4,y4]])], -1, (0,255,0), 2)
            #         x1,y1,x2,y2 = map(int,box.xyxy[0])
            #         cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)
            masks = result.masks  # Masks object for segmentation masks outputs
            if masks is None:
                msg = Vector3Stamped()
                msg.header.stamp = stamp
                msg.vector.x = float('inf')
                msg.vector.y = float('inf')
                msg.vector.z = float('inf')
            else:
                for i in range(len(masks)):
                    
                    # mask = masks[i]
                    # contour =mask.xy[0]
                    # contour = contour.astype(np.int32)
                    # contour = contour.reshape(-1, 1, 2)
                    # mask_new = np.zeros_like(frame[:,:,0])
                    # cv2.drawContours(mask_new, [contour], -1, 255, thickness=-1)
                    mask = masks.data[i].cpu().numpy()
                    mask_new = cv2.resize(mask, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
                    cv2.imshow("mask", mask_new)
                    filtered_mask = filter_main_object(mask_new)
                    contour = cv2.findContours(filtered_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0][0]
                    print(type(contour))
                    cv2.drawContours(frame, [contour], -1, (255, 0, 0))
                    rect = cv2.minAreaRect(contour)
                    print(rect)
                    msg = Vector3Stamped()
                    msg.header.stamp = stamp
                    if len(masks) > 1:
                        if i == 0:
                            width = max(rect[1])
                            x = rect[0][0]
                            y = rect[0][1]
                        elif i == 1:
                            msg.vector.x = (rect[0][0] + x )/2/12
                            msg.vector.y = (rect[0][1] + y )/2/12
                            msg.vector.z = (abs(rect[0][0] - x) + (width + max(rect[1]))/2)/12
                    else:
                        msg.vector.x = rect[0][0]/12
                        msg.vector.y = rect[0][1]/12
                        msg.vector.z = max(rect[1])/12
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    cv2.drawContours(frame, [contour], -1, (255, 0, 0))
                    cv2.drawContours(frame, [box], -1, (0, 255, 0))
            # keypoints = result.keypoints  # Keypoints object for pose outputs
            # probs = result.probs  # Probs object for classification outputs
            # obb = result.obb  # Oriented boxes object for OBB outputs
            #result.show()  # display to screen
            


        self.img_pub.publish(ros_image)
        self.position_pub.publish(msg)
        cv2.imshow("yolo", frame)
        self.key = cv2.waitKey(5)
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