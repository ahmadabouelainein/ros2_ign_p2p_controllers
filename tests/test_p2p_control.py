import os
import math
import time
import unittest
import pytest

import launch
import launch_ros
import launch_testing
from launch_testing.markers import keep_alive
import launch_testing.actions
import random
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus

from tf_transformations import quaternion_from_euler

@pytest.mark.launch_test
@keep_alive
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='ros2_ign_p2p_controllers',
            executable='P2PController',
            output={'both': 'log'}
            ),
        launch_testing.actions.ReadyToTest()
    ])


class TestP2PControllerIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_p2p_controller_node')

        cls.cmd_vel_sub = cls.node.create_subscription(
            TwistStamped,
            '/ackermann_steering_controller/reference',
            cls.cmd_callback,
            10
        )
        cls.odom_pub = cls.node.create_publisher(Odometry,
                                                 '/ackermann_steering_controller/odometry',
                                                 10)
        cls.v = 0.0
        cls.x = 0.0
        cls.y = 0.0
        cls.goal_x = 0.0
        cls.goal_y = 0.0
        cls.theta = 0.0
        cls.goal_theta = 0.0
        cls.position_reached = False
        cls.orientation_reached = False
        cls.position_tolerance = 0.2
        cls.angular_tolerance = 0.2
        cls.last_time = time.time()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def cmd_callback(cls, msg_stamped: TwistStamped):
        now = time.time()
        dt = 1/1000
        cls.last_time = now
        msg = msg_stamped.twist
        cls.v = msg.linear.x
        cls.w = msg.angular.z

        cls.x += cls.v * math.cos(cls.theta) * dt
        cls.y += cls.v * math.sin(cls.theta) * dt
        cls.theta += cls.w * dt
        cls.theta %= math.pi*2

        odom_msg = Odometry()
        odom_msg.header.stamp = rclpy.time.Time().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = cls.x
        odom_msg.pose.pose.position.y = cls.y

        q = quaternion_from_euler(0, 0, cls.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        cls.odom_pub.publish(odom_msg)

        dx = cls.goal_x - cls.x
        dy = cls.goal_y - cls.y
        pos_error = math.sqrt(dx**2 + dy**2)
        ang_error = abs(cls._angle_diff(cls.goal_theta, cls.theta))

        if pos_error < cls.position_tolerance:
            cls.position_reached = True
        if ang_error < cls.angular_tolerance:
            cls.orientation_reached = True

    @classmethod
    def _angle_diff(cls, a, b):
        diff = a - b
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def test_direction_logic(self):
        """
        Check if the vehicle drives in the correct direction based on where the goal is relative to the current position.
        """
        if (self.__class__.goal_x < self.__class__.x):
            self.assertLess(self.__class__.v < 0.0, f"Goal point is in front of current position but the vehicle is moving backwards.\n current_x: {round(self.__class__.x, 3)}\n goal_x: {round(self.__class__.goal_x, 3)}")
        if (self.__class__.goal_x > self.__class__.x):
            self.assertGreater(self.__class__.v > 0.0, f"Goal point is in behind of current position but the vehicle is moving forwards.\n current_x: {round(self.__class__.x, 3)}\n goal_x: {round(self.__class__.goal_x, 3)}")
        print(f"Vehicle direction logic is correct.\n current_x: {round(self.__class__.x, 3)}\n goal_x: {round(self.__class__.goal_x, 3)}\n v: {self.__class__.v}")
    def test_goal_reaching(self):
        """
        Send a goal and check that the simulated pose converges using Euler integration.
        """
        errors = []
        # Define the goal
        for _ in range(20):
            action_client = ActionClient(self.__class__.node, NavigateToPose, 'p2p_control')
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.pose.position.x = float(random.randint(-30, 30))
            goal_msg.pose.pose.position.y = float(random.randint(-30, 30))
            goal_msg.pose.pose.orientation.w = 1.0

            self.__class__.goal_x = goal_msg.pose.pose.position.x
            self.__class__.goal_y = goal_msg.pose.pose.position.y

            # Wait for action server
            if not action_client.wait_for_server(timeout_sec=15.0):
                self.fail("Action server not available.")

            # Send goal
            goal_future = action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self.node, goal_future)
            goal_handle = goal_future.result()
            self.assertIsNotNone(goal_handle, msg="Goal handle was not received.")
            self.assertTrue(goal_handle.accepted, msg="Goal was not accepted by the server.")

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future)  # allow more time here
            final_result = result_future.result()
            self.assertIsNotNone(final_result, msg="Result future returned None.")
            self.assertEqual(final_result.status, GoalStatus.STATUS_SUCCEEDED, msg="Goal did not succeed.")
            if not math.isclose(self.__class__.x, goal_msg.pose.pose.position.x, abs_tol=self.__class__.position_tolerance):
                errors.append(f"x mismatch: {self.__class__.x:.2f} vs {goal_msg.pose.pose.position.x:.2f}")

            if not math.isclose(self.__class__.y, goal_msg.pose.pose.position.y, abs_tol=self.__class__.position_tolerance):
                errors.append(f"y mismatch: {self.__class__.y:.2f} vs {goal_msg.pose.pose.position.y:.2f}")

            if not math.isclose(self.__class__.theta, self.__class__.goal_theta, abs_tol=self.__class__.angular_tolerance):
                errors.append(
                    f"theta mismatch: {math.degrees(self.__class__.theta):.2f}° vs {math.degrees(self.__class__.goal_theta):.2f}°"
                )
            action_client.destroy()
        if errors:
            self.fail("Goal was not reached:\n" + "\n".join(errors))
