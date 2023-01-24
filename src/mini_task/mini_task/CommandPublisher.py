import rclpy
from rclpy.node import Node
import yaml
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory


class TwistCommandPublisher(Node):
    def __init__(self, dict):
        super().__init__('command_publisher')
        self.data = dict
        self.publisher = self.create_publisher(Twist, 'diff_drive/cmd_vel', 1)
        timer = self.create_timer(1, self.timer_callback)

    def subscription_callback(self, data):
        """A callback to execute when a message is received on the specified topic."""
        pass

    def timer_callback(self):
        """A callback to execute at a specified interval."""
        move_cmd = Twist()  # Creating a message in python
        move_cmd = Twist()
        move_cmd.linear.x = self.data["linear"]["x"]
        move_cmd.angular.z = self.data["angular"]["z"]
        # Publishing the message (Usually defined data as well)
        self.publisher.publish(move_cmd)


def main(args=None):

    with open(get_package_share_directory('mini_task')+'/config/command.yaml', 'r') as file:
        command_dict = yaml.safe_load(file)
    print(command_dict)
    # Starting the node
    rclpy.init(args=args)
    node = TwistCommandPublisher(command_dict)
    rclpy.spin(node)
    # Destroy the node explicitly if Node self-terminates
    node.destroy_node()
    rclpy.shutdown()
