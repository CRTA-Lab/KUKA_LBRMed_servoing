import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rclpy.qos import qos_profile_system_default
import math


class VerticalMotion(Node):
    def __init__(self):
        super().__init__('patient_motion_node')

        self.twist_pub = self.create_publisher(TwistStamped,'/lbr/servo_node/delta_twist_cmds',qos_profile_system_default)
        
        self.i = 0
        self.create_timer(0.01,self.timer_cb)



    def timer_cb(self):
        if self.i < 1000:
            self.i += 1
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        time = msg.header.stamp.sec + msg.header.stamp.nanosec*(10**(-9))
        msg.twist.linear.z = math.cos(time)*0.3
        self.twist_pub.publish(msg)

        return
    


def main():
    rclpy.init()
    delta_twist_cmd_node = VerticalMotion()
    rclpy.spin(delta_twist_cmd_node)
    delta_twist_cmd_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()