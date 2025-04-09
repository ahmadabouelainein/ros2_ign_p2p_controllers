import unittest
from ros2_ign_p2p_controllers import P2PController
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import GoalResponse, CancelResponse
import math
from tf_transformations import euler_from_quaternion

class TestP2PController(unittest.TestCase):

    def test_multiple_goal_requests(self):
        """
        This function tests the ability of the P2PController to handle multiple goal requests simultaneously.

        Parameters:
        - node (rclpy.node.Node): The ROS2 node used for testing.
        - p2p_controller (ros2_ign_p2p_controllers.ros2_ign_p2p_controllers.P2PController): The P2PController instance to be tested.
        - executor (rclpy.executors.SingleThreadedExecutor): The executor used to spin the ROS2 node.
        - goal1 (geometry_msgs.msg.PoseStamped): The first goal pose for the P2PController to reach.
        - goal2 (geometry_msgs.msg.PoseStamped): The second goal pose for the P2PController to reach.
        - goal_handle1 (rclpy.action.server.GoalHandle): The goal handle returned by the P2PController's action server for the first goal.
        - goal_handle2 (rclpy.action.server.GoalHandle): The goal handle returned by the P2PController's action server for the second goal.

        Returns:
        - None

        The function creates a ROS2 node, P2PController instance, and executor. It then sets up two goal poses and accepts them.
        It verifies that both goals are reached by the P2PController and then succeeds the goals.
        Finally, it shuts down the executor and destroys the node.
        """
        node = Node('test_p2p_controller')
        p2p_controller = P2PController()
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        goal1 = PoseStamped()
        goal1.pose.position.x = 1.0
        goal1.pose.position.y = 2.0

        goal2 = PoseStamped()
        goal2.pose.position.x = 3.0
        goal2.pose.position.y = 4.0

        goal_handle1 = p2p_controller._action_server.take_goal()
        goal_handle2 = p2p_controller._action_server.take_goal()

        self.assertIsNotNone(goal_handle1)
        self.assertIsNotNone(goal_handle2)

        goal_handle1.accept_goal(goal1)
        goal_handle2.accept_goal(goal2)

        executor.spin_once(timeout_sec=1.0)

        self.assertTrue(p2p_controller.is_goal_reached(goal1.pose.position.x, goal1.pose.position.y))
        self.assertTrue(p2p_controller.is_goal_reached(goal2.pose.position.x, goal2.pose.position.y))

        goal_handle1.succeed()
        goal_handle2.succeed()

        executor.shutdown()
        node.destroy_node()

    def test_cancel_goal_request(self):
        """
        This function tests the cancel functionality of the P2PController.

        Parameters:
        - node (rclpy.node.Node): The ROS2 node used for testing.
        - p2p_controller (ros2_ign_p2p_controllers.ros2_ign_p2p_controllers.P2PController): The P2PController instance to be tested.
        - executor (rclpy.executors.SingleThreadedExecutor): The executor used to spin the ROS2 node.
        - goal (geometry_msgs.msg.PoseStamped): The goal pose for the P2PController to reach.
        - goal_handle (rclpy.action.server.GoalHandle): The goal handle returned by the P2PController's action server.

        Returns:
        - None

        The function creates a ROS2 node, P2PController instance, and executor. It then sets up a goal pose and accepts it.
        It verifies that the goal is not initially canceled, cancels the goal, verifies that the goal is canceled, and executes the cancel request.
        Finally, it shuts down the executor and destroys the node.
        """
        node = Node('test_p2p_controller_cancel_goal')
        p2p_controller = P2PController()
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        goal = PoseStamped()
        goal.pose.position.x = 1.0
        goal.pose.position.y = 2.0

        goal_handle = p2p_controller._action_server.take_goal()

        self.assertIsNotNone(goal_handle)

        goal_handle.accept_goal(goal)

        executor.spin_once(timeout_sec=1.0)

        self.assertFalse(goal_handle.is_cancel_requested)

        goal_handle.cancel_goal()

        executor.spin_once(timeout_sec=1.0)

        self.assertTrue(goal_handle.is_cancel_requested)

        result = goal_handle.execute_cancel_request()

        self.assertEqual(result, CancelResponse.ACCEPT)

        executor.shutdown()
        node.destroy_node()

    def test_velocity_limits(self):
        p2p_controller = P2PController()

        # Test linear velocity limits
        linear_velocity = 2.0  # m/s
        p2p_controller.k_v = 1.0  # m/s^2
        limited_velocity = p2p_controller._limit_velocity(linear_velocity)
        assert -1.5 <= limited_velocity <= 1.5, 'Linear velocity limit not respected'

        # Test angular velocity limits
        angular_velocity = 0.5  # rad/s
        p2p_controller.k_theta = 1.0  # rad/s^2
        limited_velocity = p2p_controller._limit_velocity(angular_velocity, angular=True)
        assert -1.0 <= limited_velocity <= 1.0, 'Angular velocity limit not respected'

    def _limit_velocity(self, velocity, angular=False):
        if angular:
            max_velocity = 1.0  # rad/s
        else:
            max_velocity = 1.5  # m/s
        return max(min(velocity, max_velocity), -max_velocity)
    def test_yaw_error_computation(self):
        node = Node('test_p2p_controller')
        p2p_controller = P2PController()
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        goal = PoseStamped()
        goal.pose.position.x = 1.0
        goal.pose.position.y = 2.0

        current_pose = PoseStamped()
        current_pose.pose.position.x = 0.0
        current_pose.pose.position.y = 0.0
        current_pose.pose.orientation.z = 0.0  # Assuming no rotation initially

        (_, _, theta) = euler_from_quaternion([
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w
        ])

        angle_to_goal = math.atan2(goal.pose.position.y - current_pose.pose.position.y,
                                   goal.pose.position.x - current_pose.pose.position.x)
        yaw_error = angle_to_goal - theta

        self.assertAlmostEqual(yaw_error, p2p_controller.compute_yaw_error(goal, current_pose), places=5)

        executor.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    unittest.main()