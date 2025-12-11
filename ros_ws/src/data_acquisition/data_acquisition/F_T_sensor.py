import socket
import struct
import time
import numpy as np
import matplotlib.pyplot as plt
from NetFT import Sensor

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import WrenchStamped
#from pynput import keyboard



class FTSEnsorNode(Node):
    TOOLS = {
        'sweep': [582,628,3403,0,0,4500],
        'hand_guiding': [582,628,703,0,0,4500],
    }
    def __init__(self, sensor_ip):

        FORCE_HZ = 1000.0

        super().__init__('ft_sensor_node')
        #self.sensor = Sensor(sensor_ip)
        self.zero = [0]*6
        self.sensor_com_init(sensor_ip)
        self.change_tool_transform('hand_guiding')
        self.read_calib_info()
        self.tare_sensor()
        self.get_logger().info('F/T sensor initialized')
        #self.zero = self.sensor.getMeasurement()
        self.timer = self.create_timer(1.0 / FORCE_HZ, self.timer_callback)
        self.publisher_ = self.create_publisher(WrenchStamped,'ft_sensor_data',qos_profile_system_default)

    def sensor_com_init(self,ip):
        self.sensor_com = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sensor_com.connect((ip, 49151))
        return

    def read_calib_info(self):
        comand = struct.pack('>B19B',1,*[0]*19)
        self.sensor_com.send(comand)
        msg = self.sensor_com.recv(1024)
        #print(msg)
        msg_unpacked = struct.unpack('>HBBII6H',msg)
        self.force_units = msg_unpacked[3]
        self.torque_units = msg_unpacked[4]
        self.scaling_factors = msg_unpacked[5:]
        return

    def change_tool_transform(self, tool_name='sweep'):


        comand = struct.pack('>BBB6h5B',2,4,1,*self.TOOLS[tool_name],*[0,0,0,0,0])
        self.sensor_com.send(comand)
        msg = self.sensor_com.recv(1024)
        #print(len(msg))
        #print(struct.unpack('>HBB',msg))
        return
    
    def read_FT(self):
        comand = struct.pack('>B15BII',0,*[0]*15,0,0)
        self.sensor_com.send(comand)
        msg = self.sensor_com.recv(1024)
        msg_unpacked = struct.unpack('>HHhhhhhh',msg)

        data = msg_unpacked[2:]

        fy = -data[0]/self.force_units*self.scaling_factors[0] - self.zero[1]
        fz = -data[1]/self.force_units*self.scaling_factors[1] - self.zero[2]
        fx = data[2]/self.force_units*self.scaling_factors[2] - self.zero[0]
        my = -data[3]/self.torque_units*self.scaling_factors[3] - self.zero[4]
        mz = -data[4]/self.torque_units*self.scaling_factors[4] - self.zero[5]
        mx = data[5]/self.torque_units*self.scaling_factors[5] - self.zero[3]
        return [fx,fy,fz,mx,my,mz]
    
    def tare_sensor(self,window = 10):
        if not window:
            self.zero = [0]*6
            return
        fx = 0
        fy = 0
        fz = 0
        mx = 0
        my = 0
        mz = 0
        for i in range(window):
            data = self.read_FT()
            fx += data[0]
            fy += data[1]
            fz += data[2]
            mx += data[3]
            my += data[4]
            mz += data[5]
            time.sleep(0.05)
        self.zero = [fx/window,fy/window,fz/window,mx/window,my/window,mz/window]
        return


    def timer_callback(self):
        try:
            #data = self.sensor.getMeasurement()
            data = self.read_FT()

            msg = WrenchStamped()
            msg.wrench.force.x = data[0]
            msg.wrench.force.y = data[1]
            msg.wrench.force.z = data[2]
            msg.wrench.torque.x = data[3]
            msg.wrench.torque.y = data[4]
            msg.wrench.torque.z = data[5]
            msg.header.stamp = self.get_clock().now().to_msg()

            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Sensor error: {e}")



def main():
    rclpy.init()
    sensor_ip = '192.168.0.199'

    ft_sensor_node = FTSEnsorNode(sensor_ip)
    while rclpy.ok():
        rclpy.spin_once(ft_sensor_node, timeout_sec=0.1)
    ft_sensor_node.destroy_node()
    rclpy.shutdown()
    return

if __name__ == '__main__':
    main()