import cv2
import numpy as np
import math
from random import randint
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import Vector3Stamped
from scipy.signal import butter,sosfilt_zi,sosfilt

class ConeDetectionNode(Node):
    def __init__(self):
        super().__init__('Cone_detect')
        self.position_publisher=self.create_publisher(Vector3Stamped,'phantom_position',qos_profile=qos_profile_system_default)
        self.filtered_publisher=self.create_publisher(Vector3Stamped,'phantom_position_filt',qos_profile=qos_profile_system_default)
        self.create_timer(1/30,self.find_cone)

        self.cap = cv2.VideoCapture(0)
        self.boje = [(randint(0,255),randint(0,255),randint(0,255)) for i in range(10)]
        self.data_buffer = np.zeros((150, 3))
        self.filter = butter(2, 5, btype='lowpass', output='sos', fs=30)
        zi = sosfilt_zi(self.filter)
        self.zi = np.dstack((zi, zi, zi))
        
        return

    @staticmethod
    def line_intersection(line1, line2):

                x1, y1, x2, y2 = *line1[0], *line1[1]

                x3, y3, x4, y4 = *line2[0], *line2[1]

                denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

                if denom == 0:

                    return None  # Lines are parallel

                px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom

                py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom

                return px, py
    @staticmethod
    def cart_dist(T1,T2):
        return math.sqrt((T1[0]-T2[0])**2 + (T1[1]-T2[1])**2)
    
    def filt(self):
        data, _ = sosfilt(self.filter, self.data_buffer, 0, self.zi)
        msg = Vector3Stamped()
        msg.vector.x = data[-1, 0]
        msg.vector.y = data[-1, 1]
        msg.vector.z = data[-1, 2]
        self.filtered_publisher.publish(msg)

    def find_cone(self):
        center = (350,297.5)
        ret,frame = self.cap.read()
        if not ret:
            self.position_publisher.publish(Vector3Stamped())
            return
        frame = cv2.resize(frame, (1920, 1080))
        image = frame[125:720, 645:1345]

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        _, thresholded_image = cv2.threshold(gray, 95, 255, cv2.THRESH_BINARY)
        #blurred = cv2.medianBlur(gray, 3)
        # blurred = cv2.GaussianBlur(gray,(7,7),5)


        edges = cv2.Canny(thresholded_image, 50, 150, apertureSize=3)
        
        #Detect lines using Hough Line Transform

        lines = cv2.HoughLines(edges, 1, np.pi / 180, 50,max_theta=np.pi,min_theta=0)

        if lines is not None:

            lines_mat = lines[:, 0]
            #print(lines_mat.shape)
            lines_mat = lines_mat[lines_mat[:,1].argsort()]
            #print(lines_mat)
            differences = np.diff(lines_mat[:,1])
            #print(differences)

            # Step 3: Define a threshold for significant difference (you can adjust this value)
            threshold = 10*np.pi/180

            # Step 4: Identify the boundaries where the difference exceeds the threshold
            boundaries = np.where(np.abs(differences) > threshold)[0] + 1  # Add 1 because np.diff reduces length by 1

            # Step 5: Assign group labels
            group_labels = np.zeros(len(lines_mat))
            group_id = 0
            for i in range(1, len(lines_mat)):
                if i in boundaries:
                    group_id += 1
                group_labels[i] = group_id
            #print(group_labels)
        

            # Convert polar coordinates to Cartesian and collect endpoints

            # line_coords = []

            # for rho, theta in lines_mat:

            #     a = np.cos(theta)

            #     b = np.sin(theta)

            #     x0 = a * rho

            #     y0 = b * rho

            #     x1 = int(x0 + 1000 * (-b))

            #     y1 = int(y0 + 1000 * a)

            #     x2 = int(x0 - 1000 * (-b))

            #     y2 = int(y0 - 1000 * a)

            #     line_coords.append(((x1, y1), (x2, y2)))
            
            # # Draw lines and find intersection
            
        
            # #Assuming the first two lines are the stylus edges
            # 
            try:

            #     intersection = ConeDetectionNode.line_intersection(line_coords[0], line_coords[1])

                _means = [np.array([0,0]) for i in range(int(group_labels[-1])+1)]
            # Draw the lines and intersection            
                
                for i in range(len(lines_mat)):
                    #line = line_coords[i]

                    #cv2.line(image, line[0], line[1], self.boje[int(group_labels[i])], 1)
                    _means[int(group_labels[i])] = np.vstack((_means[int(group_labels[i])],lines_mat[i,:]))
                means = []
                for i in _means:
                    i = np.delete(i,0,0)
                    means.append(np.mean(i,axis=0,dtype=np.float64))
                #print(means)
                
                has_ortog = [0 for i in range(int(group_labels[-1])+1)]
                for i in range(len(means)):
                    for j in means:
                        if abs(min(abs(means[i][1] - j[1]),2*np.pi-abs(means[i][1] - j[1]))-np.pi/2) < 10*np.pi/180:
                            has_ortog[i] = 1
                            angle = min(abs(means[i][1] - j[1]),2*np.pi-abs(means[i][1] - j[1]))
                            print(min(abs(means[i][1] - j[1]),2*np.pi-abs(means[i][1] - j[1]))*180/np.pi)
                            break
                i = 0
                ortog_lines = []
                for rho, theta in means:

                    a = np.cos(theta)

                    b = np.sin(theta)

                    x0 = a * rho

                    y0 = b * rho

                    x1 = int(x0 + 1000 * (-b))

                    y1 = int(y0 + 1000 * a)

                    x2 = int(x0 - 1000 * (-b))

                    y2 = int(y0 - 1000 * a)

                    if has_ortog[i]:
                        ortog_lines.append(((x1, y1), (x2, y2)))

                    cv2.line(image, (x1, y1), (x2, y2), (0,0,255), 3)
                    i += 1
                    # line_coords.append(((x1, y1), (x2, y2)))
                if len(ortog_lines) == 2:
                    intersection = ConeDetectionNode.line_intersection(ortog_lines[0],ortog_lines[1])
                    #cv2.circle(image, intersection, 5, (0, 0, 255), -1)
                    msg = Vector3Stamped()

                    x = intersection[0] / center[0] -1
                    y = intersection[1] / center[1] -1
                    angle = abs(angle - np.pi/2)*5
                    
                    msg.vector.x = x
                    msg.vector.y = y
                    msg.vector.z = angle
                    self.position_publisher.publish(msg)
                    self.data_buffer = np.vstack((self.data_buffer, np.array([x, y, angle])))
                    self.data_buffer = np.delete(self.data_buffer, 0, 0)
                    self.filt()

                elif len(ortog_lines) < 2:
                    self.position_publisher.publish(Vector3Stamped())
                    
                # if intersection:

                #     cv2.circle(image, intersection, 5, (0, 0, 255), -1)
            
            except Exception as e:
                print('Dogodila se greška:\n{}'.format(e))
                self.get_logger().error('Dogodila se greška:\n{}'.format(e))
                self.position_publisher.publish(Vector3Stamped())
                #return
        
        # Show the result
        # cv2.line(image, (image.shape[1]//2, 0), (image.shape[1]//2, image.shape[0]), (0,0,255), 2)
        # cv2.line(image, (image.shape[1]-150, 0), (image.shape[1]-150, image.shape[0]), (0,0,255), 2)
        # cv2.line(image, (150, 0), (150, image.shape[0]), (0,0,255), 2)

        # cv2.line(image, (0,150), (image.shape[1], 150), (0,0,255), 2)
        # cv2.line(image, (0,image.shape[0]-150), (image.shape[1], image.shape[0]-150), (0,0,255), 2)

        # cv2.line(image, (image.shape[1]//2, 0), (image.shape[1]//2, image.shape[0]), (0,0,255), 2)
        # cv2.line(image, (0,image.shape[0]//2), (image.shape[1], image.shape[0]//2), (0,0,255), 2)

        #cv2.circle(image,(int(b),int(a)),int(r),(0,255,0),2)

        cv2.imshow('Stylus with Intersection', image)
        #cv2.imshow('edges',edges)

        cv2.waitKey(1)
        return


def main():
    rclpy.init()
    node = ConeDetectionNode()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()

if __name__ == '__main__':
    main()
