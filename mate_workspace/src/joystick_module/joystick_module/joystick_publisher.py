import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class PublisherNode(Node):

    def __init__(self):
        super().__init__('publisher_node') # run Node constructor
        self.publisher_ = self.create_publisher(String, 'joystick_topic', 10) # publish to topic named first_topic
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.pygame.init()
        self.pygame.joystick.init()
        joystick = self.pygame.joystick.Joystick(0)
        joystick.init()

    def timer_callback(self):
        self.pygame.event.pump()
        left_stick_x = self.joystick.get_axis(0)  # Left stick horizontal
        left_stick_y = self.joystick.get_axis(1)  # Left stick vertical

        # Get the X and Y axes of the right stick
        right_stick_x = self.joystick.get_axis(2)  # Right stick horizontal
        right_stick_y = self.joystick.get_axis(3)  # Right stick vertical
        msg1 = String()
        msg2 = String()
        msg1.data = f"Left Stick:  X: {left_stick_x:.2f}, Y: {left_stick_y:.2f}"
        self.publisher_.publish(msg1)
        self.get_logger().info('Publishing: "%s"' % msg1.data)
        msg2.data = f"Right Stick: X: {right_stick_x:.2f}, Y: {right_stick_y:.2f}"
        self.publisher_.publish(msg2)
        self.get_logger().info('Publishing: "%s"' % msg2.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    # create publisher node obj
    publisher_node = PublisherNode()

    # run Node
    rclpy.spin(publisher_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# # joystick_publisher.py
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# import pygame
# from . import init_joysticks


# class JoystickPublisher(Node):
#     def __init__(self):
#         super().__init__('joystick_publisher')
#         self.publisher_ = self.create_publisher(Float32MultiArray, 'joystick_data', 10)
#         self.timer = self.create_timer(0.1, self.publish_joystick_data)
#         self.joysticks = init_joysticks()

#     def publish_joystick_data(self):
#         pygame.event.pump()
#         axis_values = []
#         for joystick in self.joysticks:
#             for i in range(joystick.get_numaxes()):
#                 axis_value = joystick.get_axis(i)
#                 axis_values.append(axis_value)

#         msg = Float32MultiArray(data=axis_values)
#         self.publisher_.publish(msg)
#         self.get_logger().info(f'Published joystick values: {axis_values}')


# def main(args=None):
#     rclpy.init(args=args)
#     node = JoystickPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()



