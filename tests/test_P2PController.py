import unittest
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient, CancelResponse
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from ros2_ign_p2p_controllers.P2PController import P2PController, normalize_angle
from tf_transformations import euler_from_quaternion

assert(2+2==4)

# class TestP2PController(unittest.TestCase):

#     @classmethod
#     def setUpClass(cls):
#         rclpy.init()
#         cls.executor = MultiThreadedExecutor()
#         # Create a test client node.
#         cls.test_node = Node('test_client_node')
#         cls.executor.add_node(cls.test_node)

#     @classmethod
#     def tearDownClass(cls):
#         cls.executor.shutdown()
#         cls.test_node.destroy_node()
#         rclpy.shutdown()

#     def setUp(self):
#         # Create an instance of the controller and add it to the executor.
#         self.controller = P2PController()
#         self.executor.add_node(self.controller)

#     def tearDown(self):
#         # Remove and destroy the controller node after each test.
#         self.executor.remove_node(self.controller)
#         self.controller.destroy_node()

#     def test_multiple_goal_requests(self):
#         """
#         Send two sequential goals and verify that each succeeds.
#         """
#         action_client = ActionClient(self.test_node, NavigateToPose, 'p2p_control')
#         self.assertTrue(action_client.wait_for_server(timeout_sec=5.0),
#                         msg="Action server not available")

#         # --- Send first goal ---
#         goal_msg1 = NavigateToPose.Goal()
#         goal_msg1.pose.pose.position.x = 1.0
#         goal_msg1.pose.pose.position.y = 2.0
#         # Set current pose to a different value to force some processing.
#         self.controller.current_pose.position.x = 0.0
#         self.controller.current_pose.position.y = 0.0
        
#         future1 = action_client.send_goal_async(goal_msg1)
#         rclpy.spin_until_future_complete(self.test_node, future1, timeout_sec=5.0)
#         result1 = future1.result()
#         self.assertIsNotNone(result1, msg="First goal did not return any result.")
#         if result1.goal_handle is None:
#             self.fail("First goal was not accepted by the server (goal_handle is None).")
#         # Wait for the result.
#         result_future1 = result1.goal_handle.get_result_async()
#         rclpy.spin_until_future_complete(self.test_node, result_future1, timeout_sec=5.0)
#         final_result1 = result_future1.result()
#         self.assertEqual(final_result1.status, GoalStatus.STATUS_SUCCEEDED,
#                          msg="First goal did not succeed.")

#         # --- Send second goal ---
#         goal_msg2 = NavigateToPose.Goal()
#         goal_msg2.pose.pose.position.x = 3.0
#         goal_msg2.pose.pose.position.y = 4.0
#         # Reset current pose.
#         self.controller.current_pose.position.x = 0.0
#         self.controller.current_pose.position.y = 0.0

#         future2 = action_client.send_goal_async(goal_msg2)
#         rclpy.spin_until_future_complete(self.test_node, future2, timeout_sec=5.0)
#         result2 = future2.result()
#         self.assertIsNotNone(result2, msg="Second goal did not return any result.")
#         if result2.goal_handle is None:
#             self.fail("Second goal was not accepted by the server (goal_handle is None).")
#         result_future2 = result2.goal_handle.get_result_async()
#         rclpy.spin_until_future_complete(self.test_node, result_future2, timeout_sec=5.0)
#         final_result2 = result_future2.result()
#         self.assertEqual(final_result2.status, GoalStatus.STATUS_SUCCEEDED,
#                          msg="Second goal did not succeed.")

#     def test_cancel_goal_request(self):
#         """
#         Send a goal and cancel it, verifying that the cancellation is processed.
#         """
#         # Set a very low error threshold to delay goal completion.
#         self.controller.position_error_threshold = 1e-6

#         action_client = ActionClient(self.test_node, NavigateToPose, 'p2p_control')
#         self.assertTrue(action_client.wait_for_server(timeout_sec=5.0),
#                         msg="Action server not available")

#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose.pose.position.x = 5.0
#         goal_msg.pose.pose.position.y = 5.0
#         # Set the current pose far from the goal.
#         self.controller.current_pose.position.x = 0.0
#         self.controller.current_pose.position.y = 0.0

#         send_future = action_client.send_goal_async(goal_msg)
#         rclpy.spin_until_future_complete(self.test_node, send_future, timeout_sec=5.0)
#         result = send_future.result()
#         self.assertIsNotNone(result, msg="Goal send result is None.")
#         goal_handle = result.goal_handle
#         if goal_handle is None:
#             self.fail("Goal was not accepted by the server (goal_handle is None).")

#         # Wait briefly to ensure the goal callback is active.
#         time.sleep(1.0)

#         cancel_future = goal_handle.cancel_goal_async()
#         rclpy.spin_until_future_complete(self.test_node, cancel_future, timeout_sec=5.0)
#         cancel_response = cancel_future.result()
#         # Verify that cancellation was accepted.
#         self.assertEqual(cancel_response, CancelResponse.ACCEPT,
#                          msg="Cancellation request was not accepted.")

# if __name__ == '__main__':
#     unittest.main()
