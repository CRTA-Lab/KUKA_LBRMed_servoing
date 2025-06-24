import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default,qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import WrenchStamped, TwistStamped


def saturation(value, limit = 1):
    if abs(value) > limit:
        return limit * (value / abs(value))
    return value

class ForceCompensationNode(Node):
    def __init__(self):
        super().__init__('force_compensation_node')

        self.pub_sub_cbg = ReentrantCallbackGroup()

        self.ft_sub = self.create_subscription(WrenchStamped,'/lbr/force_torque_broadcaster/wrench',self.ft_callback,qos_profile_system_default,callback_group=self.pub_sub_cbg)


    def ft_callback(self, msg: WrenchStamped):
        
        msg_out = TwistStamped()
        msg_out.header = msg.header
        msg_out.twist.linear.x = saturation(msg.wrench.force.x)
        msg_out.twist.linear.y = saturation(msg.wrench.force.y)
        msg_out.twist.linear.z = saturation(msg.wrench.force.z)
        msg_out.twist.angular.x = saturation(msg.wrench.torque.x)
        msg_out.twist.angular.y = saturation(msg.wrench.torque.y)
        msg_out.twist.angular.z = saturation(msg.wrench.torque.z)



def main():
    rclpy.init()
    node = ForceCompensationNode()
    while rclpy.ok():
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()