import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rclpy.qos import qos_profile_system_default
from tf2_ros.buffer import Buffer
from tf2_ros import TransformListener, TransformBroadcaster
import math


class VerticalMotion(Node):
    def __init__(self):
        super().__init__('patient_motion_node')

        self.declare_parameter('data_logger_path', '/home/racunalo/KUKA_LBRMed_servoing/logs/')
        data_logger_path = self.get_parameter('data_logger_path').get_parameter_value().string_value
        self.get_logger().info(f'Logging data to: {data_logger_path}')

        self.data_logger = open(data_logger_path+'/phantom_position.log', 'w')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.twist_pub = self.create_publisher(TwistStamped,'/lbr/servo_node/delta_twist_cmds',qos_profile_system_default)
        
        self.i = 0
        self.create_timer(0.01,self.timer_cb)



    def timer_cb(self):
        if self.i < 100:
            self.i += 1
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        time = msg.header.stamp.sec + msg.header.stamp.nanosec*(10**(-9))
        msg.twist.linear.z = math.cos(time)*0.3
        self.twist_pub.publish(msg)
        try:
            pose = self.tf_buffer.lookup_transform('lbr_link_0','lbr_link_ee',rclpy.time.Time())
            self.data_logger.write('TransformStamped: {}\n'.format(pose))
        except Exception as e:
            self.get_logger().error(f'Error looking up transform: {e}')

        return
    


def main():
    rclpy.init()
    delta_twist_cmd_node = VerticalMotion()
    rclpy.spin(delta_twist_cmd_node)
    delta_twist_cmd_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()