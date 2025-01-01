import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import TwistStamped,Vector3Stamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import time
#import cv2
#import numpy as np



class ServoingNode(Node):
    def __init__(self):
        time.sleep(5)
        super().__init__('LBRServoing')

        self.declare_parameter('mode','')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        self.position_publisher = self.create_publisher(TwistStamped,"/lbr/servo_node/delta_twist_cmds",qos_profile_system_default)

        self.create_subscription(Vector3Stamped,'phantom_position',self.postion_callback,qos_profile_system_default)
        self.create_subscription(Vector3Stamped,'phantom_position_filt',self.postion_filt_callback,qos_profile_system_default)

        # self._joint_trajectory_action_client = rclpy.action.ActionClient(
        #     node=self,
        #     action_type=FollowJointTrajectory,
        #     action_name="joint_trajectory_controller/follow_joint_trajectory",
        # )
        # while not self._joint_trajectory_action_client.wait_for_server(1):
        #     self.get_logger().info("Waiting for action server to become available...")
        # self.get_logger().info("Action server available.")
        return


    def postion_callback(self,msg:Vector3Stamped):
        pass

    def postion_filt_callback(self,msg:Vector3Stamped):
        x = msg.vector.x
        y = msg.vector.y
        pub_msg = TwistStamped()
        pub_msg.twist.linear.x = x/1000
        pub_msg.twist.linear.y = y/1000
        self.position_publisher.publish(pub_msg)
        return
    
    def execute(self, positions: list, sec_from_start: int = 15):
        if len(positions) != 7:
            self.get_logger().error("Invalid number of joint positions.")
            return

        joint_trajectory_goal = FollowJointTrajectory.Goal()
        goal_sec_tolerance = 1
        joint_trajectory_goal.goal_time_tolerance.sec = goal_sec_tolerance

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.time_from_start.sec = sec_from_start

        for i in range(7):
            joint_trajectory_goal.trajectory.joint_names.append(f"A{i + 1}")

        joint_trajectory_goal.trajectory.points.append(point)

        # send goal
        goal_future = self._joint_trajectory_action_client.send_goal_async(
            joint_trajectory_goal
        )
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by server.")
            return
        self.get_logger().info("Goal was accepted by server.")

        # wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=sec_from_start + goal_sec_tolerance
        )

        if (
            result_future.result().result.error_code
            != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            self.get_logger().error("Failed to execute joint trajectory.")
            return


def main():
    rclpy.init()

    node = ServoingNode()

    # if node.mode != 'hardware':
    #     node.execute(0,0,0,-1,0,1,0)
    while rclpy.ok():
        rclpy.spin_once(node,timeout_sec=0.1)
    node.destroy_node()
    return


if __name__ == '__main__':
    main()