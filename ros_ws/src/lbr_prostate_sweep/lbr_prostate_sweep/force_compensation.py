import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default,qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float32
from geometry_msgs.msg import WrenchStamped, TwistStamped
from rcl_interfaces.msg import SetParametersResult

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
        self.integral = saturation(self.integral,1)
        diff = (error-self.prev)/self.T

        self.prev = error

        controll = self.kp*error + self.ki*self.integral + self.kd*diff
        return controll

class ForceCompensationNode(Node):
    def __init__(self):
        super().__init__('force_compensation_node')

        self.pub_sub_cbg = ReentrantCallbackGroup()
        self.data_logger = open('/home/racunalo/KUKA_LBRMed_servoing/logs/force_compensation.log', 'w')


        # Only declare parameters for phi and theta PIDs (dynamic)
        self.declare_parameter('1/phi_Kp', 3.0)
        self.declare_parameter('1/phi_Ki', 20.0)
        self.declare_parameter('1/phi_Kd', 0.0)
        self.declare_parameter('1/theta_Kp', 3.0)
        self.declare_parameter('1/theta_Ki', 20.0)
        self.declare_parameter('1/theta_Kd', 0.0)

        # Fixed values for y/z PID
        self.Kp = 1/30
        self.Ki = 1/100
        self.Kd = 0.0

        # Get initial values for phi/theta PIDs
        self.phi_Kp = 1/self.get_parameter('1/phi_Kp').get_parameter_value().double_value
        self.phi_Ki = 1/self.get_parameter('1/phi_Ki').get_parameter_value().double_value
        phi_kd_val = self.get_parameter('1/phi_Kd').get_parameter_value().double_value
        self.phi_Kd = 0.0 if phi_kd_val == 0.0 else 1/phi_kd_val

        self.theta_Kp = 1/self.get_parameter('1/theta_Kp').get_parameter_value().double_value
        self.theta_Ki = 1/self.get_parameter('1/theta_Ki').get_parameter_value().double_value
        theta_kd_val = self.get_parameter('1/theta_Kd').get_parameter_value().double_value
        self.theta_Kd = 0.0 if theta_kd_val == 0.0 else 1/theta_kd_val

        self.add_on_set_parameters_callback(self.param_callback)

        self.ft_sub = self.create_subscription(WrenchStamped,'/lbr/force_torque_broadcaster/wrench',self.ft_callback,qos_profile_system_default,callback_group=self.pub_sub_cbg)
        self.twist_sub = self.create_subscription(TwistStamped,'/twist_cmd',self.twist_cb,qos_profile_system_default,callback_group=self.pub_sub_cbg)
        self.image_sub = self.create_subscription(Float32,'/axial_position',self.image_cb,qos_profile_system_default,callback_group=self.pub_sub_cbg)

        self.twist_pub = self.create_publisher(TwistStamped,'/lbr/servo_node/delta_twist_cmds',qos_profile_system_default,callback_group=self.pub_sub_cbg)

        self.twist_cmd = TwistStamped()
        self.axial_twist = 0.0


        T = 1/200
        self.xPID = PID(0.012, 0, 0.002, 1/16)
        self.yPID = PID(self.Kp, self.Ki, self.Kd, T)
        self.zPID = PID(self.Kp, self.Ki, self.Kd, T)
        self.phi_PID = PID(self.phi_Kp, self.phi_Ki, self.phi_Kd, T)
        self.theta_PID = PID(self.theta_Kp, self.theta_Ki, self.theta_Kd, T)

    def param_callback(self, params):
        for param in params:
            if param.name == '1/phi_Kp':
                self.phi_Kp = 1/param.value
                self.phi_PID.kp = self.phi_Kp
            elif param.name == '1/phi_Ki':
                self.phi_Ki = 1/param.value
                self.phi_PID.ki = self.phi_Ki
            elif param.name == '1/phi_Kd':
                if param.value == 0.0:
                    self.phi_Kd = 0.0
                else:
                    self.phi_Kd = 1/param.value
                self.phi_PID.kd = self.phi_Kd
            elif param.name == '1/theta_Kp':
                self.theta_Kp = 1/param.value
                self.theta_PID.kp = self.theta_Kp
            elif param.name == '1/theta_Ki':
                self.theta_Ki = 1/param.value
                self.theta_PID.ki = self.theta_Ki
            elif param.name == '1/theta_Kd':
                if param.value == 0.0:
                    self.theta_Kd = 0.0
                else:
                    self.theta_Kd = 1/param.value
                self.theta_PID.kd = self.theta_Kd
        return SetParametersResult(successful=True)

    def image_cb(self, msg: Float32):
        self.get_logger().info('Image position: {}'.format(msg.data))
        self.axial_twist = self.xPID.update(msg.data)
        
    def ft_callback(self, msg: WrenchStamped):
        #self.get_logger().info('Force: {}'.format(msg.wrench.force.y))
        msg_out = TwistStamped()
        msg_out.header = msg.header
        #msg_out.twist.linear.x = saturation(self.xPID.update(msg.wrench.force.x) + self.twist_cmd.twist.linear.x)
        msg_out.twist.linear.x = self.axial_twist
        msg_out.twist.linear.y = saturation(self.yPID.update(msg.wrench.force.y - 6) + self.twist_cmd.twist.linear.y)
        msg_out.twist.linear.z = saturation(self.zPID.update(msg.wrench.force.z) + self.twist_cmd.twist.linear.z)
        msg_out.twist.angular.x = self.twist_cmd.twist.angular.x

        M_y = msg.wrench.torque.y - msg.wrench.force.z * 0.33849
        M_z = msg.wrench.torque.z + msg.wrench.force.x * 0.06569 - msg.wrench.force.y * 0.33849

        self.get_logger().info('Moment: measured: {}, corrected: {},  Force : {}'.format(msg.wrench.torque.z, M_z,msg.wrench.force.y), throttle_duration_sec=0.5)
        # msg_out.twist.angular.y = saturation(self.phi_PID.update(M_y*4/3) + self.twist_cmd.twist.angular.y)
        # msg_out.twist.angular.z = saturation(self.theta_PID.update(M_z*4/3) + self.twist_cmd.twist.angular.z)

        self.data_logger.write('WrenchStamped: {} \t Corrected Moment: {}, {}\n'.format(msg, M_y, M_z))

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