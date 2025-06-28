import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray

class HardwareInterface(Node):
    def __init__(self):
        super().__init__('so101_hardware_interface')
        self.get_logger().info('SO-101 Hardware Interface started.')
        self.get_logger().info('This node is a placeholder for real robot communication.')
        self.get_logger().info('Modify this file to connect to your robot\'s SDK.')

        # Subscription to the command topic from the teleoperation node
        self.command_subscription = self.create_subscription(
            PoseArray,
            'so101_teleop/cmd_pose',
            self.command_callback,
            10)

    def command_callback(self, msg):
        """
        This callback receives commands from the teleoperation system.
        Here, you would translate these commands and send them to the real robot.
        """
        left_pose = msg.poses[0]
        right_pose = msg.poses[1]

        # --- REPLACE WITH YOUR ROBOT'S SDK CALLS ---
        # Example: send_robot_command('left_arm', left_pose)
        # Example: send_robot_command('right_arm', right_pose)
        
        self.get_logger().info(
            f"Received command: Left Arm Pose: P({left_pose.position.x:.2f}, {left_pose.position.y:.2f}, {left_pose.position.z:.2f})",
            throttle_duration_sec=1.0)
        
        # Here you would also read the real robot's state (joint angles, etc.)
        # and publish it to the /joint_states topic for RViz and other nodes.

def main(args=None):
    rclpy.init(args=args)
    hardware_interface = HardwareInterface()
    rclpy.spin(hardware_interface)
    hardware_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 