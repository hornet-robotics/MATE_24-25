import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VectorDrive(Node):

    def __init__(self):
        super().__init__('vector_drive_node') # run Node constructor

        # setup subscriber
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'joystick_topic', # subscribe to joystick topic
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):

        joystick_left_x = msg.data[0]
        joystick_left_y = msg.data[1]

        joystick_right_x = msg.data[2]
        joystick_right_y = msg.data[3]

        left_trigger = msg.data[4]
        right_trigger = msg.data[5]

        # scale to match pwm
        pwm_scale = 127.5
        centering_constant = 127.5 # add to values to make center the 0 value
        # 2d
        forward_back = pwm_scale * joystick_left_y + centering_constant
        strafe = (pwm_scale * -joystick_left_x) + centering_constant
        yaw = (pwm_scale * -joystick_right_x) + centering_constant
        # 3d
        up_down = pwm_scale * (right_trigger - left_trigger) + centering_constant
        pitch = pwm_scale * joystick_right_y + centering_constant

        # give values to motors

        #          front
        #           +x
        #     m0 //--m5--\\ m3
        #  +y       ----
        #     m1 \\--m4--// m2
        # counter-clockwise is positive for yaw rotation
        # nose up is positive in pitch rotation

        m0 = int(forward_back - strafe - yaw)
        m1 = int(forward_back + strafe - yaw)
        m2 = int(forward_back - strafe + yaw)
        m3 = int(forward_back + strafe + yaw)
        m4 = int(up_down - pitch)
        m5 = int(up_down + pitch)

        # TODO set motor pwm values here

        self.get_logger().info('From joystick_topic I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    # create subscriber node obj
    vector_drive_node = VectorDrive()

    # run Node
    rclpy.spin(vector_drive_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vector_drive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()