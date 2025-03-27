import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import TwistStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
import math 
from tf_transformations import euler_from_quaternion

class P2PController(Node):
    def __init__(self):
        super().__init__('p2p_controller')
        self.get_logger().info('P2P controller with Lyapunov control initialized')
        
        self.publisher = self.create_publisher(TwistStamped, '/ackermann_steering_controller/reference', 10, callback_group=ReentrantCallbackGroup())
        self.odom_subscriber = self.create_subscription(Odometry, '/ackermann_steering_controller/odometry', self.odom_callback, 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.current_pose = Pose()
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'p2p_control',
            self.goal_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self.position_error_threshold = 0.1  # meters
        self.yaw_error_threshold = 0.02  # radians

        # Control gains for Lyapunov-based controller
        self.k_v = 0.5  # Linear velocity gain
        self.k_theta = 0.5  # Heading correction gain
        self.k_y = -2  # Lateral error gain (negative for stability)

    def goal_callback(self, goal_handle):
        self.get_logger().info('Received goal request')
        goal = goal_handle.request.pose 
        
        iteration_count = 0
        while not goal_handle.is_cancel_requested:
            ex = goal.pose.position.x - self.current_pose.position.x 
            ey = goal.pose.position.y - self.current_pose.position.y 
            e_d = math.sqrt(ex**2 + ey**2)  # Euclidean distance error

            (_, _, theta) = euler_from_quaternion([
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            ])

            # Compute angle to goal
            angle_to_goal = math.atan2(ey, ex)
            yaw_error = normalize_angle(angle_to_goal - theta)

            # Decide whether to move forward or reverse
            forward = abs(yaw_error) > math.pi / 2  # If goal is behind, reverse

            if forward:
                yaw_error = normalize_angle(yaw_error + math.pi)  # Adjust yaw error
                e_d *= -1  # Move backward instead of forward

            e_y = -ex * math.sin(theta) + ey * math.cos(theta)  # Lateral error

            # Lyapunov control law with direction adjustment
            v = self.k_v * e_d
            omega = self.k_y * e_y + self.k_theta * yaw_error

            # Limit velocities
            v = max(min(v, 1.5), -1.5)
            omega = max(min(omega, 1.0), -1.0)

            # Publish velocity command
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.twist.linear.x = v
            twist_msg.twist.angular.z = omega
            self.publisher.publish(twist_msg)

            # Provide feedback
            feedback_msg = NavigateToPose.Feedback()
            feedback_msg.current_pose.pose = self.current_pose
            goal_handle.publish_feedback(feedback_msg)

            iteration_count += 1
            if iteration_count % 100 == 0:
                direction = "FORWARD" if forward else "REVERSE"
                self.get_logger().info(
                    f'[{direction}] Euclidean Error: {e_d:.3f}m, Ex: {ex:.3f}m, Ey: {ey:.3f}m, Yaw Error: {math.degrees(yaw_error):.2f}°'
                )
                iteration_count = 0
            if self.is_goal_reached(abs(e_d)):  # Use absolute error for stopping
                break

        # Stop the robot
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.angular.z = 0.0
        self.publisher.publish(twist_msg)

        if goal_handle.is_cancel_requested:
            return CancelResponse.ACCEPT

        # Return result
        goal_handle.succeed()
        result_msg = NavigateToPose.Result()
        return result_msg


    def odom_callback(self, msg):
        # Update current_pose based on odometry data
        self.current_pose = msg.pose.pose

    def is_goal_reached(self, error):
        if error < self.position_error_threshold:
            self.get_logger().info('Goal reached')
            return True
        return False

def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def main(args=None):
    rclpy.init(args=args)
    p2p_controller = P2PController()
    executor = MultiThreadedExecutor()
    executor.add_node(p2p_controller)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
