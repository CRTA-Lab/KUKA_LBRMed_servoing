import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import Vector3Stamped
import cv2
import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi

class CircleDetectionNode(Node):
    def __init__(self):
        super().__init__('CircleDetection')
        cb_timer = ReentrantCallbackGroup()
        cb_pub = ReentrantCallbackGroup()

        self.position_publisher = self.create_publisher(Vector3Stamped, 'phantom_position', qos_profile_system_default)
        self.filtered_publisher = self.create_publisher(Vector3Stamped, 'phantom_position_filt', qos_profile_system_default)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('No video stream')
        self.create_timer(1/30, self.img_read, callback_group=cb_timer)
        self.data_buffer = np.zeros((150, 3))
        self.filter = butter(3, 3, btype='lowpass', output='sos', fs=30)
        zi = sosfilt_zi(self.filter)
        self.zi = np.dstack((zi, zi, zi))
        
        # Create trackbars for dynamic adjustment
        # cv2.namedWindow('Trackbars')
        # cv2.createTrackbar('Min Dist', 'Trackbars', 200, 500, self.on_trackbar_change)
        # cv2.createTrackbar('Param 1', 'Trackbars', 50, 100, self.on_trackbar_change)
        # cv2.createTrackbar('Param 2', 'Trackbars', 28, 100, self.on_trackbar_change)
        # cv2.createTrackbar('Min Radius', 'Trackbars', 50, 100, self.on_trackbar_change)
        # cv2.createTrackbar('Max Radius', 'Trackbars', 100, 200, self.on_trackbar_change)
        

    def info(self, msg: str):
        self.get_logger().info(msg)
        return

    def on_trackbar_change(self, val):
        # Function to handle changes in trackbar values (nothing to do here, just for live update)
        pass

    def img_read(self):
        px_to_mm = (1520 - 635) / 50
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Could not read the frame, skipping...')
        frame = cv2.resize(frame, (1920, 1080))
        image = frame[110:900, 635:1520]
        image = image[10:image.shape[0] - 30, 10:image.shape[1] - 30]
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Preprocess: Blur and detect edges
        blurred = cv2.medianBlur(gray, 7)

        # Read values from trackbars
        # minDist = cv2.getTrackbarPos('Min Dist', 'Trackbars')
        # param1 = cv2.getTrackbarPos('Param 1', 'Trackbars')
        # param2 = cv2.getTrackbarPos('Param 2', 'Trackbars')
        # minRadius = cv2.getTrackbarPos('Min Radius', 'Trackbars')
        # maxRadius = cv2.getTrackbarPos('Max Radius', 'Trackbars')

        minDist = 500
        param1 = 26
        param2 = 24
        minRadius = 65
        maxRadius = 100

        # Hough Transform to detect circles with dynamic parameters
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=minDist,
            param1=param1,
            param2=param2,
            minRadius=minRadius,
            maxRadius=maxRadius
        )

        # Validate and fit circles to semicircles
        if circles is not None:
            circles = np.uint16(np.around(circles))
            best_cover = 0
            best_circle = [0, 0, 0]
            for circle in circles[0, :]:
                x, y, r = circle
                if r > best_cover:
                    best_cover = r
                    best_circle = circle
        else:
            self.info('no circle')
            # Show the image
            cv2.imshow('slika', image)
            cv2.waitKey(1)
            return

        x, y, r = best_circle
        cv2.circle(image, (x, y), r, (0, 255, 0), 2)

        # Publish detected circle data
        msg = Vector3Stamped()
        x = x / px_to_mm
        y = y / px_to_mm
        r = r / px_to_mm
        msg.vector.x = x
        msg.vector.y = y
        msg.vector.z = r
        self.position_publisher.publish(msg)

        # Show the image
        cv2.imshow('slika', image)
        cv2.waitKey(1)

        # Add data to the buffer and apply filtering
        self.data_buffer = np.vstack((self.data_buffer, np.array([x, y, r])))
        self.data_buffer = np.delete(self.data_buffer, 0, 0)
        self.filt()

    def filt(self):
        data, _ = sosfilt(self.filter, self.data_buffer, 0, self.zi)
        msg = Vector3Stamped()
        msg.vector.x = data[-1, 0]
        msg.vector.y = data[-1, 1]
        msg.vector.z = data[-1, 2]
        self.filtered_publisher.publish(msg)

def main():
    rclpy.init()
    node = CircleDetectionNode()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()

if __name__ == '__main__':
    main()
