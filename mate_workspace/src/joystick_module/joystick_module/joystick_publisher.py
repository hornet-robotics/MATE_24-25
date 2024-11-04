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
        pygame.init() 
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

    def timer_callback(self):
        pygame.event.pump()
        result= [] 
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
        msg.data = result #Msg Value=[Left_x stick,Left_y stick,Right_x stick,Right_y stick]
        self.publisher_.publish(msg)
        # display message sent
        self.get_logger().info(f'Publishing: {msg.data}')


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