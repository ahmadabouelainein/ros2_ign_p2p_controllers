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

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus

from tf_transformations import quaternion_from_euler

from builtin_interfaces.msg import Time as TimeMsg

@pytest.mark.launch_test
@keep_alive
def generate_test_description():
    # Launch the controller node
    # file_path = os.path.dirname(__file__)
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='ros2_ign_p2p_controllers',
            executable='P2PController',
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestP2PControllerIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_integration_node')

        cls.cmd_vel_sub = cls.node.create_subscription(
            TwistStamped,
            '/ackermann_steering_controller/reference',
            cls.cmd_callback,
            10
        )
        cls.odom_pub = cls.node.create_publisher(Odometry,
                                                 '/ackermann_steering_controller/odometry',
                                                 10)

        cls.action_client = ActionClient(cls.node, NavigateToPose, 'p2p_control')
        cls.x = 0.0
        cls.y = 0.0
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
        dt = now - cls.last_time
        cls.last_time = now
        msg = msg_stamped.twist
        v = msg.linear.x
        w = msg.angular.z

        cls.x += v * math.cos(cls.theta) * dt
        cls.y += v * math.sin(cls.theta) * dt
        cls.theta += w * dt

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

    def test_goal_reaching(self):
        """
        Send a goal and check that the simulated pose converges using Euler integration.
        """
        # Define the goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = 1.0
        goal_msg.pose.pose.position.y = 2.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.__class__.goal_x = goal_msg.pose.pose.position.x
        self.__class__.goal_y = goal_msg.pose.pose.position.y

        # Wait for action server
        if not self.action_client.wait_for_server(timeout_sec=15.0):
            self.fail("Action server not available.")

        # Send goal
        goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, goal_future, timeout_sec=5.0)
        goal_handle = goal_future.result()
        self.assertIsNotNone(goal_handle, msg="Goal handle was not received.")
        self.assertTrue(goal_handle.accepted, msg="Goal was not accepted by the server.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=30.0)  # allow more time here
        final_result = result_future.result()
        self.assertIsNotNone(final_result, msg="Result future returned None.")
        self.assertEqual(final_result.status, GoalStatus.STATUS_SUCCEEDED, msg="Goal did not succeed.")

    

        # msgp = (f"Controller did not reach the goal. Position is not correct"
        #        f"\nCurrent position: ({self.__class__.x:.2f}, {self.__class__.y:.2f}, "
        #        f"\nGoal position: ({goal_msg.pose.pose.position.x:.2f}, {goal_msg.pose.pose.position.y:.2f}, ")
        # msgo = (f"Controller did not reach the goal. Heading is not correct "
        #        f"\nCurrent heading: "
        #        f"{math.degrees(self.__class__.theta):.2f})"
        #        f"\nGoal heading: "
        #        f"{math.degrees(self.__class__.goal_theta):.2f}")
        # msg = (f"Controller did not reach the goal within timeout. "
        #        f"\nCurrent position: ({self.__class__.x:.2f}, {self.__class__.y:.2f}, "
        #        f"\nGoal position: ({goal_msg.pose.pose.position.x:.2f}, {goal_msg.pose.pose.position.y:.2f}, "
        #        f"\nCurrent heading: "
        #        f"{math.degrees(self.__class__.theta):.2f})"
        #        f"\nGoal heading: "
        #        f"{math.degrees(self.__class__.goal_theta):.2f}")
        # self.assertTrue((self.__class__.orientation_reached and self.__class__.position_reached), msg)
        # self.assertAlmostEqual(self.__class__.x, goal_msg.pose.pose.position.x, places=1, msg=msgp)
        # self.assertAlmostEqual(self.__class__.y, goal_msg.pose.pose.position.y, places=1, msg=msgp)
        # self.assertAlmostEqual(self.__class__.theta, self.__class__.goal_theta, places=1, msg=msgo)
        errors = []

        if not math.isclose(self.__class__.x, goal_msg.pose.pose.position.x, abs_tol=self.__class__.position_tolerance):
            errors.append(f"x mismatch: {self.__class__.x:.2f} vs {goal_msg.pose.pose.position.x:.2f}")

        if not math.isclose(self.__class__.y, goal_msg.pose.pose.position.y, abs_tol=self.__class__.position_tolerance):
            errors.append(f"y mismatch: {self.__class__.y:.2f} vs {goal_msg.pose.pose.position.y:.2f}")

        if not math.isclose(self.__class__.theta, self.__class__.goal_theta, abs_tol=self.__class__.angular_tolerance):
            errors.append(
                f"theta mismatch: {math.degrees(self.__class__.theta):.2f}° vs {math.degrees(self.__class__.goal_theta):.2f}°"
            )

        if errors:
            self.fail("Goal was not reached:\n" + "\n".join(errors))
