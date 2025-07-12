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


class PID():
    def __init__(self,Kp,Ki,Kd,T):
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        self.T = T
        self.integral = 0
        self.prev = 0
        return
    
    def update(self,error):
        self.integral += error*self.T
        if self.integral > 1:
            self.integral = 1
        elif self.integral > -1:
            self.integral = -1
        diff = (error-self.prev)/self.T

        self.prev = error

        controll = self.kp*error + self.ki*self.integral + self.kd*diff
        return controll

class ForceCompensationNode(Node):
    def __init__(self):
        super().__init__('force_compensation_node')

        self.pub_sub_cbg = ReentrantCallbackGroup()

        self.ft_sub = self.create_subscription(WrenchStamped,'/lbr/force_torque_broadcaster/wrench',self.ft_callback,qos_profile_system_default,callback_group=self.pub_sub_cbg)
        self.twist_sub = self.create_subscription(TwistStamped,'/twist_cmd',self.twist_cb,qos_profile_system_default,callback_group=self.pub_sub_cbg)

        self.twist_pub = self.create_publisher(TwistStamped,'/lbr/servo_node/delta_twist_cmds',qos_profile_system_default,callback_group=self.pub_sub_cbg)

        self.twist_cmd = TwistStamped()

        T = 1/200
        Kp = 1/6
        Ki = 1/15
        Kd = 0

        self.xPID = PID(Kp,Ki,Kd,T)
        self.yPID = PID(Kp,Ki,Kd,T)
        self.zPID = PID(Kp,Ki,Kd,T)

    def ft_callback(self, msg: WrenchStamped):
        self.get_logger().info('Force: {}'.format(msg.wrench.force.y))
        msg_out = TwistStamped()
        msg_out.header = msg.header
        #msg_out.twist.linear.x = saturation(self.xPID.update(msg.wrench.force.x) + self.twist_cmd.twist.linear.x)
        msg_out.twist.linear.x = self.twist_cmd.twist.linear.x
        msg_out.twist.linear.y = saturation(self.yPID.update(msg.wrench.force.y - 2) + self.twist_cmd.twist.linear.y)
        msg_out.twist.linear.z = saturation(self.zPID.update(msg.wrench.force.z) + self.twist_cmd.twist.linear.z)
        msg_out.twist.angular.x = self.twist_cmd.twist.angular.x
        msg_out.twist.angular.y = self.twist_cmd.twist.angular.y
        msg_out.twist.angular.z = self.twist_cmd.twist.angular.z

        self.twist_pub.publish(msg_out)
    
    def twist_cb(self,msg: TwistStamped):
        self.twist_cmd = msg
        return



def main():
    rclpy.init()
    node = ForceCompensationNode()
    while rclpy.ok():
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()