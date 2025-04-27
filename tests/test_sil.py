import unittest
import pytest
import launch
import random
import launch_testing
from launch_testing.markers import keep_alive
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import random
import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
@pytest.mark.launch_test
@keep_alive
def generate_test_description():
    return launch.LaunchDescription([

    IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ros2_ign_p2p_controllers'),
                    'launch',
                    'simulator.launch.py'
                ])
            ),
            launch_arguments={
                'headless': 'true'
            }.items(),
        ), 
    launch_testing.actions.ReadyToTest()
    ])


class TestSiL(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_sil_integration_node')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_goal_reaching_sil(self):
        """
        Send a goal and check that the robot drives to the goal pose.
        """
        # Define the goal
        for _ in range(20):
            action_client = ActionClient(self.__class__.node, NavigateToPose, 'p2p_control')
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.pose.position.x = float(random.randint(-30, 30))
            goal_msg.pose.pose.position.y = float(random.randint(-30, 30))
            goal_msg.pose.pose.orientation.w = 1.0
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
            rclpy.spin_until_future_complete(self.node, result_future)  
            final_result = result_future.result()
            self.assertIsNotNone(final_result, msg="Result future returned None.")
            self.assertEqual(final_result.status, GoalStatus.STATUS_SUCCEEDED, msg="Goal did not succeed.")
            action_client.destroy()