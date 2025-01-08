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

        #cv2.namedWindow("Controls")
 
        # # Create trackbars
        # cv2.createTrackbar("Block Size", "Controls", 11, 200,self.on_trackbar_change)
        # cv2.createTrackbar("C", "Controls", 5, 50,self.on_trackbar_change)
        # cv2.createTrackbar("Min Area", "Controls", 200, 2000,self.on_trackbar_change)
        
        # # Create trackbars for dynamic adjustment
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
    
    def img_read_bbox(self):
        px_to_mm = (1520 - 635) / 50
        ret, original = self.cap.read()
        # original = cv2.resize(original, (1920, 1080))
        original = original[110:720, 635:1350]
        # original = original[10:original.shape[0]-30, 10:original.shape[1]-30]
        # Get values from the trackbars
        block_size = cv2.getTrackbarPos("Block Size", "Controls") | 1  # Must be odd
        c = cv2.getTrackbarPos("C", "Controls")
        min_area = cv2.getTrackbarPos("Min Area", "Controls")

        # block_size = 45
        # c = 11
        # min_area = 2000

        # Convert the image to HSV and threshold it
        hsv = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (15, 50, 50), (30, 255, 255))
    
        # Apply adaptive thresholding
        gray = cv2.cvtColor(original, cv2.COLOR_BGR2GRAY)
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, 
                                    cv2.THRESH_BINARY_INV, block_size, c)
    
        # Morphological opening to clean up noise
        element = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        clean_mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, element)
    
        # Find contours
        contours, _ = cv2.findContours(clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
        # Draw contours and bounding boxes
        result = original.copy()
        flag = True
        for contour in contours:
            if cv2.contourArea(contour) > min_area:
                flag = False
                color = tuple(np.random.randint(0, 256, 3).tolist())
                cv2.drawContours(result, [contour], -1, color, 2)
                bounding_rect = cv2.boundingRect(contour)
                cv2.rectangle(result, bounding_rect, color, 2)
                size = bounding_rect[2] - bounding_rect[0]
                center = (bounding_rect[0] + bounding_rect[2] // 2, bounding_rect[1] + bounding_rect[2] // 2)
                cv2.circle(result, center, 5, color, -1)
            
        if flag:return
        # Display the results
        
        msg = Vector3Stamped()
        x = (bounding_rect[0] + bounding_rect[2] // 2) / px_to_mm
        y = (bounding_rect[1] + bounding_rect[2] // 2) / px_to_mm
        r = (bounding_rect[2]) / px_to_mm
        msg.vector.x = x
        msg.vector.y = y
        msg.vector.z = r
        self.position_publisher.publish(msg)
        self.data_buffer = np.vstack((self.data_buffer, np.array([x, y, r])))
        self.data_buffer = np.delete(self.data_buffer, 0, 0)
        self.filt()
        #cv2.imshow("Original", original)
        #cv2.imshow("Thresholded", clean_mask)
        cv2.imshow("Contours", result)
        cv2.waitKey(1)

    def img_read(self):
        px_to_mm = (1520 - 635) / 50
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Could not read the frame, skipping...')
        frame = cv2.resize(frame, (1920, 1080))
        image = frame[125:720, 645:1345]
        #image = image[10:image.shape[0] - 30, 10:image.shape[1] - 30]
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Preprocess: Blur and detect edges
        blurred = cv2.medianBlur(gray, 5)

        # Read values from trackbars
        # minDist = cv2.getTrackbarPos('Min Dist', 'Trackbars')
        # param1 = cv2.getTrackbarPos('Param 1', 'Trackbars')
        # param2 = cv2.getTrackbarPos('Param 2', 'Trackbars')
        # minRadius = cv2.getTrackbarPos('Min Radius', 'Trackbars')
        # maxRadius = cv2.getTrackbarPos('Max Radius', 'Trackbars')

        minDist = 500
        param1 = 77
        param2 = 34
        minRadius = 70
        maxRadius = 180

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
            msg = Vector3Stamped()
            msg.vector.x = 0
            msg.vector.y = 0
            msg.vector.z = 0
            self.position_publisher.publish(msg)
            # Show the image
            cv2.imshow('slika', image)
            cv2.waitKey(1)
            return

        x, y, r = best_circle
        cv2.circle(image, (x, y), r, (0, 255, 0), 2)

        # Publish detected circle data
        msg = Vector3Stamped()
        x = (x / 350) - 1
        y = (y / 297.5) - 1
        r = r / 200
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


    def img_read_combo(self):
        px_to_mm = (1520 - 635) / 50
        ret, original = self.cap.read()
        # original = cv2.resize(original, (1920, 1080))
        original = original[110:610, 635:1350]
        # original = original[10:original.shape[0]-30, 10:original.shape[1]-30]
        # Get values from the trackbars
        block_size = cv2.getTrackbarPos("Block Size", "Controls") | 1  # Must be odd
        c = cv2.getTrackbarPos("C", "Controls")
        min_area = cv2.getTrackbarPos("Min Area", "Controls")

        # block_size = 45
        # c = 11
        # min_area = 2000

        # Convert the image to HSV and threshold it
        hsv = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (15, 50, 50), (30, 255, 255))
    
        # Apply adaptive thresholding
        gray = cv2.cvtColor(original, cv2.COLOR_BGR2GRAY)
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, 
                                    cv2.THRESH_BINARY_INV, block_size, c)
        

        #Read values from trackbars
        minDist = cv2.getTrackbarPos('Min Dist', 'Trackbars')
        param1 = cv2.getTrackbarPos('Param 1', 'Trackbars')
        param2 = cv2.getTrackbarPos('Param 2', 'Trackbars')
        minRadius = cv2.getTrackbarPos('Min Radius', 'Trackbars')
        maxRadius = cv2.getTrackbarPos('Max Radius', 'Trackbars')

        # minDist = 500
        # param1 = 26
        # param2 = 24
        # minRadius = 65
        # maxRadius = 100

        # Hough Transform to detect circles with dynamic parameters
        circles = cv2.HoughCircles(
            thresh,
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
            element = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            clean_mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, element)
            cv2.imshow('slika', original)
            cv2.imshow("Thresholded", clean_mask)
            cv2.waitKey(1)
            return

        x, y, r = best_circle
        cv2.circle(original, (x, y), r, (0, 255, 0), 2)

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
        element = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        clean_mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, element)
        cv2.imshow('slika', original)
        cv2.imshow("Thresholded", clean_mask)
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
