# joystick_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class JoystickSubscriber(Node):
    def __init__(self):
        super().__init__('joystick_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'joystick_data',
            self.joystick_callback,
            10
        )

    def joystick_callback(self, msg):
        axis_values = msg.data
        self.get_logger().info(f'Received joystick values: {axis_values}')

def main(args=None):
    rclpy.init(args=args)
    node = JoystickSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
