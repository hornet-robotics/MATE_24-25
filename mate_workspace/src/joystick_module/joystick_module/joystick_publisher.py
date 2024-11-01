import pygame
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class PublisherNode(Node):

    def __init__(self):
        super().__init__('publisher_node') # run Node constructor
        self.publisher_ = self.create_publisher(Float32MultiArray, 'joystick_topic', 10) # publish to topic named first_topic
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0
        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

    def timer_callback(self):
        pygame.event.pump()
        result= [] #Format [Left_x,Left_y,Right_x,Right_y]
        left_stick_x = self.joystick.get_axis(0)  # Left stick horizontal
        result.append(left_stick_x)
        left_stick_y = self.joystick.get_axis(1)  # Left stick vertical
        result.append(left_stick_y)

        # Get the X and Y axes of the right stick
        right_stick_x = self.joystick.get_axis(3)  # Right stick horizontal
        result.append(right_stick_x)
        right_stick_y = self.joystick.get_axis(4)  # Right stick vertical
        result.append(right_stick_y)
        msg = Float32MultiArray()
        msg.data = result
        self.publisher_.publish(msg)
        # display message sent
        self.get_logger().info(f'Publishing: {msg.data}')
        #self.i += 1


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



