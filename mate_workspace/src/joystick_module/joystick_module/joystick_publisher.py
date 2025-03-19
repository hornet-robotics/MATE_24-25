import sys

import pygame
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class PublisherNode(Node):

    def __init__(self):
        super().__init__('publisher_node') # run Node constructor
        self.publisher_ = self.create_publisher(Float32MultiArray, 'joystick_topic', 10) # publish to topic named first_topic
        timer_period = 0  # seconds
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

        trigger_press_left = self.joystick.get_axis(2) #How much left trigger is pressed
        result.append(trigger_press_left)
        trigger_press_right = self.joystick.get_axis(5) #How much right trigger is pressed
        result.append(trigger_press_right)

        dpad_x = self.joystick.get_hat(0)[0] #X-direciton of dpad
        result.append(dpad_x)
        dpad_y = self.joystick.get_hat(0)[1] #Y-direction of dpad
        result.append(dpad_y)

        button_press_a=self.joystick.get_button(0) #X on Playstation, A on Xbox
        result.append(button_press_a)
        button_press_b = self.joystick.get_button(1) #Circle on Playstation, B on Xbox
        result.append(button_press_b)
        button_press_y = self.joystick.get_button(2) #Triangle on Playstation, Y on Xbox
        result.append(button_press_y)
        button_press_x = self.joystick.get_button(3) #Square on Playstation, X on Xbox
        result.append(button_press_x)
        button_press_lbumper = self.joystick.get_button(4) #Left Bumper
        result.append(button_press_lbumper)
        button_press_rbumper = self.joystick.get_button(5) #Right Bumper
        result.append(button_press_rbumper)

        if self.joystick.get_button(10)==1: #If PS Button or Guide Button is Pressed, Quits The Program
            sys.exit()

        msg = Float32MultiArray()
        msg.data = result #Msg Value=[Left_x stick,Left_y stick,Right_x stick,Right_y stick, Left trigger strength, right trigger strength, X-direciton of dpad, Y-direction of dpad, (X on Playstation, A on Xbox), (Circle on Playstation, B on Xbox), (Triangle on Playstation, Y on Xbox), (Square on Playstation, X on Xbox), Left Bumper, Right Bumper]
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