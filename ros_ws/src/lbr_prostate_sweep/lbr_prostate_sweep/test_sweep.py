import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class TestSweep(Node):

    def __init__(self):
        super().__init__('test_sweep_node')
        self.twist_pub = self.create_publisher(TwistStamped,'/lbr/servo_node/delta_twist_cmds', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lbr_link_ee'

        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 1.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.twist_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = TestSweep()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()