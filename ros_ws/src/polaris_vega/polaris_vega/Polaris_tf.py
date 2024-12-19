import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sksurgerynditracker.nditracker import NDITracker
import ndicapy
#import socket
#import datetime
import numpy as np
import numpy.linalg as lin
from scipy import optimize
from scipy.spatial.transform import Rotation
from scipy.signal import butter,sosfilt,sosfilt_zi
from custom_interfaces.msg import MarkerTf
from geometry_msgs.msg import Transform



class Polaris_tf(Node):
    def __init__(self):
        super().__init__('Polaris_tf')

        self.tf_pub = self.create_publisher(Transform,'fixed_marker_tf',qos_profile_sensor_data)
        self.tracker = self.camera_init()
        self.framenumber = 0
        
        self.pub_timer = self.create_timer(0.02,self.timer_callback)

    
    def camera_init(self):
        '''
        Configures the connection to the camera and initiation of the tracking

        Args : none

        Output : NDITracker - camera comunication object
        '''
        SETTINGS = {
            "tracker type": "vega",
            "ip address": "192.168.40.100",
            "romfiles" : ["markeri/8700339.rom",
                        "markeri/FEM0001.rom",
                        "markeri/8700338.rom"],
            
            }
        TRACKER = NDITracker(SETTINGS)

        ndicapy.ndiCommand(TRACKER._device,'SET Param.Tracking.Track Frequency=2')
        

        TRACKER.start_tracking()
        return TRACKER
    
    def timer_callback(self):
        port_handles, timestamps, framenumbers, tracking, quality = self.tracker.get_frame()
        if self.framenumber == framenumbers[0]:
            return
        
        msg = Transform()
        marker = tracking[0]
        msg.translation.x = marker[0,3]
        msg.translation.y = marker[1,3]
        msg.translation.z = marker[2,3]

        rot = Rotation.from_matrix(marker[:3,:3])
        quat = rot.as_quat()

        msg.rotation.w = quat[0]
        msg.rotation.x = quat[1]
        msg.rotation.y = quat[2]
        msg.rotation.z = quat[3]

        self.tf_pub.publish(msg)
        return
    

def main():
    rclpy.init()

    node = Polaris_tf()

    while rclpy.ok():
        rclpy.spin_once(node)
    
    rclpy.shutdown()
    return

if __name__ == '__main__':
    main()



