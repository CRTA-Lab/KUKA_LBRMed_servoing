import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import Vector3Stamped
import cv2
import numpy as np
#from scipy.optimize import least_squares
from scipy.signal import butter,sosfilt,sosfilt_zi


class CircleDetectionNode(Node):
    def __init__(self):
        super().__init__('CircleDetection')
        cb_timer = ReentrantCallbackGroup()
        cb_pub = ReentrantCallbackGroup()

        self.position_publisher = self.create_publisher(Vector3Stamped,'phantom_position',qos_profile_system_default)
        self.filtered_publisher = self.create_publisher(Vector3Stamped,'phantom_position_filt',qos_profile_system_default)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('No video stream')
        self.create_timer(1/30,self.img_read,callback_group=cb_timer)
        self.data_buffer = np.zeros((150,3))
        self.filter = butter(3,3,btype='lowpass',output='sos',fs=30)
        zi = sosfilt_zi(self.filter)
        self.zi = np.dstack((zi,zi,zi))
        return

    def info(self,msg:str):
        self.get_logger().info(msg)
        return

    def img_read(self):
        #cap = cv2.VideoCapture(0)
        px_to_mm = (1520-635)/50
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Could not read the frame, skiping...')
        frame = cv2.resize(frame, (1920, 1080))
        #print(frame.shape)
        image = frame[110:900, 635:1520]
        image = image[10:image.shape[0]-10, 10:image.shape[1]-10]
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Preprocess: Blur and detect edges
        #blurred = cv2.GaussianBlur(gray, (5, 5), 7)
        blurred = cv2.medianBlur(gray,5)
        edges = cv2.Canny(blurred, 50, 120)

        # Hough Transform to detect circles
        circles = cv2.HoughCircles(
            blurred, 
            cv2.HOUGH_GRADIENT, 
            dp=1, 
            minDist=200, 
            param1=50, 
            param2=28, 
            minRadius=60, 
            maxRadius=150
        )

        # Validate and fit circles to semicircles
        if circles is not None:
            circles = np.uint16(np.around(circles))
            best_cover = 0
            best_circle = [0,0,0]
            for circle in circles[0, :]:
                x, y, r = circle
                # Draw the detected circle
                #cv2.circle(image, (x, y), r, (0, 255, 0), 2)
                
                # Check how much of the circle matches the edges
                mask = np.zeros_like(gray)
                cv2.circle(mask, (x, y), r, 255, 1)
                overlap = cv2.bitwise_and(edges, edges, mask=mask)
                coverage_ratio = np.sum(overlap > 0) / (2 * np.pi * r)
                
                # if coverage_ratio > 0.3:  # Adjust threshold as needed
                #     print(f"Circle detected at ({x}, {y}) with radius {r} and coverage {coverage_ratio:.2f}")
                # else:
                #     print(f"Partial circle discarded at ({x}, {y}) with low coverage.")

                if r > best_cover:
                    best_cover = r
                    best_circle = circle
        else:return
        #print(best_cover)
        #print(best_circle)
        x, y, r = best_circle
        cv2.circle(image, (x, y), r, (0, 255, 0), 2)
        msg = Vector3Stamped()

        x = x/px_to_mm
        y = y/px_to_mm
        r = r/px_to_mm

        msg.vector.x = x
        msg.vector.y = y
        msg.vector.z = r
        
        self.position_publisher.publish(msg)
        #cv2.imshow('slika',image)
        #cv2.waitKey(1)
        self.data_buffer = np.vstack((self.data_buffer,np.array([x,y,r])))
        self.data_buffer = np.delete(self.data_buffer,0,0)
        self.filt()
        return
    
    def filt(self):
        data,zo = sosfilt(self.filter,self.data_buffer,0,self.zi)
        
        msg = Vector3Stamped()

        msg.vector.x = data[-1,0]
        msg.vector.y = data[-1,1]
        msg.vector.z = data[-1,2]

        self.filtered_publisher.publish(msg)
        return
        



def main():
    rclpy.init()

    node = CircleDetectionNode()
    while rclpy.ok():
        rclpy.spin_once(node,timeout_sec=0.1)
    node.destroy_node()
    return


if __name__ == '__main__':
    main()