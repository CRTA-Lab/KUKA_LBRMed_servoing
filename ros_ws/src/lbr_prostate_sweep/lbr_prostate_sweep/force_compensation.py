import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default,qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from geometry_msgs.msg import WrenchStamped, TwistStamped, Vector3Stamped
from rcl_interfaces.msg import SetParametersResult

def saturation(value, limit = 1):
    '''Saturates the value to the specified limit.
    Args:        value (float): The value to be saturated.
                 limit (float): The saturation limit.
    Returns:     float: The saturated value.'''
    if abs(value) > limit:
        return limit * (value / abs(value))
    return value


class PID():
    '''A simple discreet PID controller class.'''
    def __init__(self,Kp,Ki,Kd,T):
        '''Initializes the PID controller with the specified parameters
        Args:        Kp (float): proportional gain
                     Ki (float): integral gain
                     Kd (float): derivative gain
                     T (float): sample time
        Returns:     None'''
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        self.T = T
        self.integral = 0
        self.prev = 0
        return
    
    def update(self,error):
        '''Updates the PID controller with the controlled value error.
        Args:        error (float): new error value
        Returns:     float: control output'''

        if abs(error) < 0.02:
            return 0.0

        # discrete integral
        self.integral += error*self.T
        # saturating the value for anti-windup
        self.integral = saturation(self.integral,1)

        # discrete derivative
        diff = (error-self.prev)/self.T
        self.prev = error

        controll = self.kp*error + self.ki*self.integral + self.kd*diff
        return saturation(controll)
    
    def reset(self):
        '''Resets the PID controller.'''
        self.integral = 0
        self.prev = 0
        return
    

class ForceCompensationNode(Node):
    '''Node for force compensation using PID control.'''
    def __init__(self):
        super().__init__('force_compensation_node')

        self.pub_sub_cbg = ReentrantCallbackGroup()

        self.declare_parameter('data_logger_path', '/home/racunalo/KUKA_LBRMed_servoing/logs/')
        data_logger_path = self.get_parameter('data_logger_path').get_parameter_value().string_value
        self.get_logger().info(f'Logging data to: {data_logger_path}')

        self.data_logger = open(data_logger_path+'force_compensation.log', 'w')
        

        # declare parameters for phi and theta PIDs (dynamic) (writen by copilot)
        self.declare_parameter('1/phi_Kp', 5.0)
        self.declare_parameter('1/phi_Ki', 30.0)
        self.declare_parameter('1/phi_Kd', 0.0)
        self.declare_parameter('1/theta_Kp', 5.0)
        self.declare_parameter('1/theta_Ki', 30.0)
        self.declare_parameter('1/theta_Kd', 0.0)

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

        # Fixed values for y/z PID

        self.Kp = 1/25
        self.Ki = 1/100
        self.Kd = 1/250
        self.Kd = 0


        T = 1/200
        self.xPID_lin = PID(0.01, 0, 0.002, 1/16)
        self.xPID_conv = PID(0.009, 0, 0.001, 1/20)
        self.xPID = PID(self.Kp, self.Ki, self.Kd, T)
        self.yPID = PID(self.Kp, self.Ki, self.Kd, T)
        self.zPID = PID(self.Kp, self.Ki, self.Kd, T)
        self.phi_PID = PID(self.phi_Kp, self.phi_Ki, self.phi_Kd, T)
        self.theta_PID = PID(self.theta_Kp, self.theta_Ki, self.theta_Kd, T)

        self.declare_parameter('1/Kp', 30.0)
        self.declare_parameter('1/Ki', 150.0)
        self.declare_parameter('1/Kd', 0.0)

        # self.Kp = 1/self.get_parameter('1/Kp').get_parameter_value().double_value
        # self.Ki = 1/self.get_parameter('1/Ki').get_parameter_value().double_value
        # Kd_val = self.get_parameter('1/Kd').get_parameter_value().double_value
        # self.Kd = 0.0 if Kd_val == 0.0 else 1/Kd_val
        
        


        # create subscribers and publishers
        #self.ft_sub = self.create_subscription(WrenchStamped,'/lbr/force_torque_broadcaster/wrench',self.ft_callback,qos_profile_system_default,callback_group=self.pub_sub_cbg)
        self.ft_sub = self.create_subscription(WrenchStamped,'/ft_sensor_data',self.ft_callback,qos_profile_system_default,callback_group=self.pub_sub_cbg)
        self.twist_sub = self.create_subscription(TwistStamped,'/twist_cmd',self.twist_cb,qos_profile_system_default,callback_group=self.pub_sub_cbg)
        self.image_sub = self.create_subscription(Float32,'/axial_position',self.image_cb,qos_profile_system_default,callback_group=self.pub_sub_cbg)
        self.conv_data_sub = self.create_subscription(Vector3Stamped,'/conv_element_position',self.conv_data_cb,qos_profile_system_default,callback_group=self.pub_sub_cbg)

        self.twist_pub = self.create_publisher(TwistStamped,'/lbr/servo_node/delta_twist_cmds',qos_profile_system_default,callback_group=self.pub_sub_cbg)

        self.client_start_sweep=self.create_client(Trigger, "/sweep/start")
        self.create_service(Trigger,'/sweep/initialise',self.start_sweep,qos_profile=qos_profile_system_default)

        #Start Trigger init
        # Create a Trigger request to start rotation
        self.client_start_sweep.wait_for_service()
        self.get_logger().info('Connected to /sweep/start service.')
        

        # initialize variables
        self.twist_cmd = TwistStamped()
        self.axial_twist = 0.0
        self.use_linear_for_x = True
        self.initialised = False
        self.initialise = False

        self.conv_x = 0
        self.conv_y = 0
        self.conv_w = 0
        self.conv_r = 0
        
        self.hand_guiding_mode = True

        # initialize PID controllers
        
    def param_callback(self, params):
        '''Callback for dynamic parameter changes. (written by copilot)
        Args:        params (list): list of changed parameters
        Returns:     SetParametersResult: result of the callback'''
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
            elif param.name == '1/Kp':
                self.Kp = 1/param.value
                self.yPID.kp = self.Kp
                self.zPID.kp = self.Kp
            elif param.name == '1/Ki':
                if param.value == 0.0:
                    self.Ki = 0.0
                else:
                    self.Ki = 1/param.value
                self.yPID.ki = self.Ki
                self.zPID.ki = self.Ki
            elif param.name == '1/Kd':
                if param.value == 0.0:
                    self.Kd = 0.0
                else:
                    self.Kd = 1/param.value
                self.yPID.kd = self.Kd
                self.zPID.kd = self.Kd

        return SetParametersResult(successful=True)


    def start_sweep(self,request:Trigger.Request,response:Trigger.Response):
        self.hand_guiding_mode = False
        response.success = True
        return response

    def image_cb(self, msg: Float32):
        '''Callback for the position calculated from image subscriber.
        Args:        msg (Float32): message containing the position of prostate from img
        Returns:     None'''
        #self.get_logger().info('Image position: {}'.format(msg.data))
        if not self.use_linear_for_x: return
        if not self.initialised and abs(msg.data) <= 0.1:
            self.initialise = True
            self.use_linear_for_x = False
            self.axial_twist = 0.0
            self.get_logger().info('Image position initialised')
            request = Trigger.Request()
            self.get_logger().info('Start sweep.')                 
            #future = self.client_start_sweep.call_async(request)
        else:
            self.axial_twist = self.xPID_lin.update(msg.data)
            if self.axial_twist == 0.0:
                self.use_linear_for_x = False
            self.get_logger().info('Using Axial element')
        return
    
    def conv_data_cb(self, msg: Vector3Stamped):
        '''Callback for the position calculated from image subscriber.
        Args:        msg (Vector3Stamped): message containing the position of prostate from img
        Returns:     None'''
        if not self.initialised:
            if self.initialise:
                self.initialised = True
                self.conv_x = msg.vector.x
                self.conv_y = msg.vector.y
                self.conv_w = msg.vector.z
                self.conv_r = ((msg.vector.y-2)**2 + (msg.vector.x-490/12)**2)**0.5
            return
        #self.get_logger().info('Radius: {}, init radius: {}'.format(((msg.vector.y-2)**2 + (msg.vector.x-490/12)**2)**0.5,self.conv_r))
        #((msg.vector.y-2)**2 + (msg.vector.x-490/12)**2)**0.5 - self.conv_r
        if msg.vector.x == float('inf') or abs(msg.vector.z) < 25:
            self.use_linear_for_x = True
            self.xPID_lin.reset()
        elif not self.use_linear_for_x:
            
            self.axial_twist = self.xPID_conv.update(self.conv_w - msg.vector.z)
            self.get_logger().info('Using Convex element')
            self.get_logger().info('Convex pid output : {}'.format(self.axial_twist),throttle_duration_sec=0.5)
        else:
            self.use_linear_for_x = False
            self.xPID_conv.reset()
        return

    def ft_callback(self, msg: WrenchStamped):
        '''Callback for the force-torque sensor data.
        Args:        msg (WrenchStamped): message containing the force-torque data
        Returns:     None'''

        #self.get_logger().info('Force: {}'.format(msg.wrench.force.y))
        msg_out = TwistStamped()
        msg_out.header = msg.header
        #msg_out.twist.linear.x = saturation(self.xPID.update(msg.wrench.force.x) + self.twist_cmd.twist.linear.x)
        if not self.hand_guiding_mode:
            msg_out.twist.linear.x = self.axial_twist
            msg_out.twist.linear.y = saturation(self.yPID.update(msg.wrench.force.y - 7) + self.twist_cmd.twist.linear.y)
            msg_out.twist.linear.z = saturation(self.zPID.update(msg.wrench.force.z) + self.twist_cmd.twist.linear.z)
            msg_out.twist.angular.x = self.twist_cmd.twist.angular.x
        else:
            msg_out.twist.linear.x = self.xPID.update(msg.wrench.force.x)
            msg_out.twist.linear.y = self.yPID.update(msg.wrench.force.y)
            msg_out.twist.linear.z = self.zPID.update(msg.wrench.force.z)


        # M_y = msg.wrench.torque.y - msg.wrench.force.z * 0.33849
        # M_z = msg.wrench.torque.z + msg.wrench.force.x * 0.06569 - msg.wrench.force.y * 0.33849

        #self.get_logger().info('Moment: measured: {}, corrected: {},  Force : {}'.format(msg.wrench.torque.z, M_z,msg.wrench.force.y), throttle_duration_sec=0.5)
        msg_out.twist.angular.y = saturation(self.phi_PID.update(msg.wrench.torque.y) + self.twist_cmd.twist.angular.y)
        msg_out.twist.angular.z = saturation(self.theta_PID.update(msg.wrench.torque.z) + self.twist_cmd.twist.angular.z)

        # self.data_logger.write('{}.{:09d}, {}, {}, {}, {}, {}, {}\n'.format(msg.header.stamp.sec,msg.header.stamp.nanosec,
        #                                                                                             msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z,
        #                                                                                             msg.wrench.torque.x,msg.wrench.torque.y,msg.wrench.torque.z,))

        self.twist_pub.publish(msg_out)
    
    def twist_cb(self,msg: TwistStamped): 
        '''Callback for forwarding other twist commands.
        Args:        msg (TwistStamped): message containing the twist command
        Returns:     None'''
        self.twist_cmd = msg
        return



def main():
    rclpy.init()
    node = ForceCompensationNode()
    while rclpy.ok():
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()