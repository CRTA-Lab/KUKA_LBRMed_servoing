import sys
import time
import threading
import queue
import platform
import csv
import datetime
import bisect
import matplotlib.pyplot as plt
from NetFT import Sensor

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
#from pynput import keyboard



class FTSEnsorNode(Node):
    def __init__(self, sensor_ip):

        FORCE_HZ = 1000.0

        super().__init__('ft_sensor_node')
        self.sensor = Sensor(sensor_ip)
        self.zero = self.sensor.getMeasurement()
        self.timer = self.create_timer(1.0 / FORCE_HZ, self.timer_callback)
        self.publisher_ = self.create_publisher(WrenchStamped,'ft_sensor_data')

    def timer_callback(self):
        try:
            data = self.sensor.getMeasurement()
            fx = (data[0] - self.zero[0]) / 1e6
            fy = (data[1] - self.zero[1]) / 1e6
            fz = (data[2] - self.zero[2]) / 1e6
            mx = (data[3] - self.zero[3]) / 1e6
            my = (data[4] - self.zero[4]) / 1e6
            mz = (data[5] - self.zero[5]) / 1e6

            msg = WrenchStamped()
            msg.wrench.force.x = fx
            msg.wrench.force.y = fy
            msg.wrench.force.z = fz
            msg.wrench.torque.x = mx
            msg.wrench.torque.y = my
            msg.wrench.torque.z = mz
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