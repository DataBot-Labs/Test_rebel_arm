import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class PositionCommandPublisher(Node):
    def __init__(self):
        super().__init__('position_command_publisher')
        # Publisher for the forward position controller
        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.timer = self.create_timer(0.5, self.publish_command)  # Timer for periodic publishing

    def publish_command(self):
        msg = Float64MultiArray()
        # Command positions for the six joints
        msg.data = [0.5, 1.0, -0.5, 1.2, 0.0, -1.0]  # Replace with desired positions
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing commands: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = PositionCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
