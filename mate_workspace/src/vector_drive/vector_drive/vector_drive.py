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

        self.radius = 0 # TODO add value here


    def listener_callback(self, msg):

        joystick_left_x = msg.data[0]
        joystick_left_y = msg.data[1]

        joystick_right_x = msg.data[2]
        joystick_right_y = msg.data[3]

        left_trigger = msg.data[4]
        right_trigger = msg.data[5]

        # scale to match pwm
        pwm_scale = 1
        # 2d
        forward_back = pwm_scale * joystick_left_y
        strafe = pwm_scale * -joystick_left_x
        yaw = pwm_scale * -joystick_right_x
        # 3d
        up_down = pwm_scale * (right_trigger - left_trigger) # TODO adjust trigger values to make sense
        pitch = pwm_scale * joystick_right_y

        # give values to motors

        #          front
        #           +x
        #     m0 //--m5--\\ m3
        #  +y       ----
        #     m1 \\--m4--// m2

        m0 = forward_back - strafe -(2 * self.radius * yaw) # verify turning logic here
        m1 = forward_back + strafe -(2 * self.radius * yaw)
        m2 = forward_back + strafe +(2 * self.radius * yaw)
        m3 = forward_back - strafe +(2 * self.radius * yaw)
        m4 = up_down - (2 * self.radius * pitch)
        m5 = up_down + (2 * self.radius * pitch)

        # TODO add motor values to motors.setvalue


        # display message received
        self.get_logger().info('Heard: "%s"' % msg.data)


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