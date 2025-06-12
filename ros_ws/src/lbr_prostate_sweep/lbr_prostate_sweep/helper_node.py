import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory


class HelperNode(Node):
    '''Node for translating underining lbr topics and actions for moveit
    Extends Node'''
    def __init__(self):
        super().__init__('Helper_node')
        cb_grup = ReentrantCallbackGroup()
        self.create_subscription(JointState, '/lbr/joint_states', self.joint_state_callback, qos_profile_system_default,callback_group=cb_grup)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', qos_profile_system_default,callback_group=cb_grup)

        self.action_client = ActionClient(self,FollowJointTrajectory,'/lbr/joint_trajectory_controller/follow_joint_trajectory',callback_group=cb_grup)
        self.action_server = ActionServer(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory', self.execute_callback,callback_group=cb_grup)

        self.result = None
        
        return
 
    def joint_state_callback(self, msg: JointState):
        '''Callback function for the joint state subscriber
        Args : self, msg - JointState message
        Returns: None
        '''
        # publishing the joint states to the topic
        self.joint_state_pub.publish(msg)
        return
    
    def execute_callback(self, goal:ServerGoalHandle):
        self.goal_handle = goal
        self.future = self.action_client.send_goal_async(goal.request)
        self.future.add_done_callback(self.goal_response_callback)
        rclpy.spin_until_future_complete(self, self.future)
        goal.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

        
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('Getting result...')
        self.result = future.result()

    def feedback_callback(self, feedback:FollowJointTrajectory):
        self.get_logger().info('{}'.format(type(feedback)))
        self.goal_handle.publish_feedback(feedback)

def main():
    rclpy.init()
    node = HelperNode()
    while rclpy.ok():
        rclpy.spin_once(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()