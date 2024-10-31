# joystick_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import pygame
from . import init_joysticks


class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'joystick_data', 10)
        self.timer = self.create_timer(0.1, self.publish_joystick_data)
        self.joysticks = init_joysticks()

    def publish_joystick_data(self):
        pygame.event.pump()
        axis_values = []
        for joystick in self.joysticks:
            for i in range(joystick.get_numaxes()):
                axis_value = joystick.get_axis(i)
                axis_values.append(axis_value)

        msg = Float32MultiArray(data=axis_values)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published joystick values: {axis_values}')


def main(args=None):
    rclpy.init(args=args)
    node = JoystickPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



