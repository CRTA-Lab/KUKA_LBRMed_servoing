import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import Vector3Stamped
from scipy.signal import butter,sosfilt_zi,sosfilt

class ConeDetectionNode(Node):
    '''Class for the comunication with the US probe and finding the cones lines on the picture
    Extends Node'''
    def __init__(self):
        super().__init__('Cone_detect')
        # publisher for raw found data
        self.position_publisher=self.create_publisher(Vector3Stamped,'phantom_position',qos_profile=qos_profile_system_default)
        # publisher for filtered data
        self.filtered_publisher=self.create_publisher(Vector3Stamped,'phantom_position_filt',qos_profile=qos_profile_system_default)

        #timer for periodicaly reading the US image
        self.create_timer(1/46,self.find_cone)

        # connecting to the camera
        self.cap = cv2.VideoCapture(0)

        # setup of filter and its data buffer
        self.data_buffer = np.zeros((150, 3))
        self.filter = butter(3, 3, btype='lowpass', output='sos', fs=46)
        zi = sosfilt_zi(self.filter)
        self.zi = np.dstack((zi, zi, zi))
        
        return

    @staticmethod
    def line_intersection(line1, line2):
        '''Function for calculatinf the interesection point between two lines in cartesian coordinate sys

        Args : 
                line1 - two points defining the first line (tuple((x1,y1),(x2,y2)))
                line1 - two points defining the second line (tuple((x1,y1),(x2,y2)))
        Out :
                px - x coordinate of intersection
                py - y coordinate of intersection'''

        x1, y1, x2, y2 = *line1[0], *line1[1]

        x3, y3, x4, y4 = *line2[0], *line2[1]

        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

        if denom == 0:

            return None  # Lines are parallel

        px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom

        py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom

        return px, py

    
    def filt(self):
        '''Method for appying the filter to the buffered data and publising the newst data to the appropriate topic
        
        Args : 
                self
        Out :
                none'''
        data, _ = sosfilt(self.filter, self.data_buffer, 0, self.zi)
        msg = Vector3Stamped()
        msg.vector.x = data[-1, 0]
        msg.vector.y = data[-1, 1]
        msg.vector.z = data[-1, 2]
        self.filtered_publisher.publish(msg)


    def find_cone(self):
        '''Method for finding the cone on the picture and publising the data to the appropriate topic
        Method finds blobs on the picture and assumes that the largest one where two separate lines can be found is the cone
        
        Args : 
                self
        Out :
                none'''
        center = (350,297.5)
        ret,frame = self.cap.read()
        if not ret:
            self.position_publisher.publish(Vector3Stamped())
            self.get_logger().warn('No picture')
            return
        
        # resizing the frame and croping out are with US picture
        frame = cv2.resize(frame, (1920, 1080))
        image = frame[125:720, 645:1345]

        # manipulating the image to find the blobs and subsequently lines
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        _, thresholded_image = cv2.threshold(gray, 95, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(thresholded_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)

        # if no contours are found diplay the immages exit the method
        if not len(contours):
            cv2.imshow('Stylus with Intersection', image)
            cv2.imshow('mask',thresholded_image)
            cv2.waitKey(1)
            return
        try:
            
            for cnt in sorted_contours:
                blank = np.zeros(thresholded_image.shape, dtype=np.uint8)
                cv2.drawContours(blank,[cnt],-1,255,-1)
                #blank = cv2.cvtColor(blank, cv2.COLOR_BGR2GRAY)


                edges = cv2.Canny(blank, 50, 150, apertureSize=3)
                
                #Detect lines using Hough Line Transform

                lines = cv2.HoughLines(edges, 1, np.pi / 180, 50,max_theta=np.pi,min_theta=0)

                if lines is not None:

                    lines_mat = lines[:, 0]
                    lines_mat = lines_mat[lines_mat[:,1].argsort()]
                    differences = np.diff(lines_mat[:,1])

                    threshold = 10*np.pi/180

                    boundaries = np.where(np.abs(differences) > threshold)[0] + 1  # Add 1 because np.diff reduces length by 1

                    group_labels = np.zeros(len(lines_mat))
                    group_id = 0
                    for i in range(1, len(lines_mat)):
                        if i in boundaries:
                            group_id += 1
                        group_labels[i] = group_id
                

                    
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
                    if len(means) != 2:continue
                    angle = min(abs(means[0][1] - means[1][1]),2*np.pi-abs(means[0][1] - means[1][1]))
                    
                    
                    lines_cart = []
                    for rho, theta in means:

                        a = np.cos(theta)

                        b = np.sin(theta)

                        x0 = a * rho

                        y0 = b * rho

                        x1 = int(x0 + 1000 * (-b))

                        y1 = int(y0 + 1000 * a)

                        x2 = int(x0 - 1000 * (-b))

                        y2 = int(y0 - 1000 * a)

                        #if has_ortog[i]:
                        lines_cart.append(((x1, y1), (x2, y2)))

                        cv2.line(image, (x1, y1), (x2, y2), (0,0,255), 3)
                    intersection = ConeDetectionNode.line_intersection(lines_cart[0],lines_cart[1])
                    #cv2.circle(image, intersection, 5, (0, 0, 255), -1)
                    msg = Vector3Stamped()

                    x = intersection[0] / center[0] -1
                    y = intersection[1] / center[1] -1
                    angle = abs(angle - np.pi/2)
                    
                    msg.vector.x = x
                    msg.vector.y = y
                    msg.vector.z = angle
                    self.position_publisher.publish(msg)
                    self.data_buffer = np.vstack((self.data_buffer, np.array([x, y, angle])))
                    self.data_buffer = np.delete(self.data_buffer, 0, 0)
                    self.filt()
                    break
                    

            cv2.imshow('edges',edges)
            cv2.imshow('mask',blank)
            cv2.waitKey(1)
            
        except Exception as e:
            print('Dogodila se greška:\n{}'.format(e))
            self.get_logger().error('Dogodila se greška:\n{}'.format(e))
            self.position_publisher.publish(Vector3Stamped())
            cv2.imshow('Stylus with Intersection', image)
            cv2.waitKey(1)
            #return
        

        

        
        return


    def find_cone_old(self):
        '''Method for finding the cone on the picture and publising the data to the appropriate topic
        Method finds blobs on the picture and assumes that the largest one where two separate lines can be found is the cone
        
        Args : 
                self
        Out :
                none'''
        center = (350,297.5)
        ret,frame = self.cap.read()
        if not ret:
            self.position_publisher.publish(Vector3Stamped())
            self.get_logger().warn('No picture')
            return
        
        # resizing the frame and croping out are with US picture
        frame = cv2.resize(frame, (1920, 1080))
        image = frame[125:720, 645:1345]

        # manipulating the image to find the blobs and subsequently lines
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        _, thresholded_image = cv2.threshold(gray, 95, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(thresholded_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)

        # if no contours are found diplay the immages exit the method
        if not len(contours):
            cv2.imshow('Stylus with Intersection', image)
            cv2.imshow('mask',thresholded_image)
            cv2.waitKey(1)
            return
        try:
            
            for cnt in sorted_contours:
                blank = np.zeros(thresholded_image.shape, dtype=np.uint8)
                cv2.drawContours(blank,[cnt],-1,255,-1)
                #blank = cv2.cvtColor(blank, cv2.COLOR_BGR2GRAY)


                edges = cv2.Canny(blank, 50, 150, apertureSize=3)
                
                #Detect lines using Hough Line Transform

                lines = cv2.HoughLines(edges, 1, np.pi / 180, 50,max_theta=np.pi,min_theta=0)

                if lines is not None:

                    lines_mat = lines[:, 0]
                    lines_mat = lines_mat[lines_mat[:,1].argsort()]
                    differences = np.diff(lines_mat[:,1])

                    threshold = 10*np.pi/180

                    boundaries = np.where(np.abs(differences) > threshold)[0] + 1  # Add 1 because np.diff reduces length by 1

                    group_labels = np.zeros(len(lines_mat))
                    group_id = 0
                    for i in range(1, len(lines_mat)):
                        if i in boundaries:
                            group_id += 1
                        group_labels[i] = group_id
                

                    
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
                    if len(means) != 2:continue
                    #i = 0
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

                        #if has_ortog[i]:
                        ortog_lines.append(((x1, y1), (x2, y2)))

                        cv2.line(image, (x1, y1), (x2, y2), (0,0,255), 3)
                        i += 1
                        # line_coords.append(((x1, y1), (x2, y2)))
                    if len(means) == 2:
                        intersection = ConeDetectionNode.line_intersection(ortog_lines[0],ortog_lines[1])
                        #cv2.circle(image, intersection, 5, (0, 0, 255), -1)
                        msg = Vector3Stamped()

                        x = intersection[0] / center[0] -1
                        y = intersection[1] / center[1] -1
                        angle = abs(angle - np.pi/2)
                        
                        msg.vector.x = x
                        msg.vector.y = y
                        msg.vector.z = angle
                        self.position_publisher.publish(msg)
                        self.data_buffer = np.vstack((self.data_buffer, np.array([x, y, angle])))
                        self.data_buffer = np.delete(self.data_buffer, 0, 0)
                        self.filt()
                        break
                    

                    # elif len(ortog_lines) < 2:
                    #     self.position_publisher.publish(Vector3Stamped())
                        
                    # if intersection:

                    #     cv2.circle(image, intersection, 5, (0, 0, 255), -1)
            cv2.imshow('Stylus with Intersection', image)
            cv2.imshow('edges',edges)
            cv2.imshow('mask',blank)
            cv2.waitKey(1)
            
        except Exception as e:
            print('Dogodila se greška:\n{}'.format(e))
            self.get_logger().error('Dogodila se greška:\n{}'.format(e))
            self.position_publisher.publish(Vector3Stamped())
            cv2.imshow('Stylus with Intersection', image)
            cv2.waitKey(1)
            #return
        

        

        
        return


def main():
    rclpy.init()
    node = ConeDetectionNode()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()

if __name__ == '__main__':
    main()
