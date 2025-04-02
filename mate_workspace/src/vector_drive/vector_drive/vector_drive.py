import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import serial

arduino_port = '/dev/ttyACM0'
baud_rate = 9600

ser = serial.Serial(arduino_port, baud_rate)


class VectorDrive(Node):

    def __init__(self):
        super().__init__('vector_drive_node')  # run Node constructor

        # setup subscriber
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'joystick_topic',  # subscribe to joystick topic
            self.listener_callback,
            10)

    def listener_callback(self, msg):

        #            front
        #             + y
        #              |
        #              |
        #              |
        #              |
        #    ------------------ +x
        #              |
        #              |
        #              |
        #              |
        # clockwise is positive for yaw rotation
        # nose (front) up is positive in pitch rotation

        #          front
        #     m0 //--m4--\\ m3
        #           ----
        #     m1 \\--m5--// m2

        # reverse y output to make up + instead of -
        joystick_left_x = msg.data[0]
        joystick_left_y = -msg.data[1]

        joystick_right_x = msg.data[2]
        joystick_right_y = -msg.data[3]

        left_trigger = msg.data[4]
        right_trigger = msg.data[5]

        # scale to match pwm
        pwm_scale = 499.5
        centering_constant = 499.5  # add to values to make center the 0 value
        # 2d
        forward_back = joystick_left_y
        strafe = joystick_left_x
        yaw = joystick_right_x
        # 3d
        up_down = right_trigger - left_trigger
        pitch = joystick_right_y

        # give values to motors
        #  scale from joystick sya to pwm sys   clip correct range    center is true 0
        m0 = int(pwm_scale * self.clip(forward_back + strafe + yaw) + centering_constant)
        m1 = int(pwm_scale * self.clip(forward_back - strafe + yaw) + centering_constant)
        m2 = int(pwm_scale * self.clip(forward_back + strafe - yaw) + centering_constant)
        m3 = int(pwm_scale * self.clip(forward_back - strafe - yaw) + centering_constant)
        m4 = int(pwm_scale * self.clip(up_down + pitch) + centering_constant)
        m5 = int(pwm_scale * self.clip(up_down - pitch) + centering_constant)

        # ADDING SEGMENT TO CONVERT M0-M5 TO STRING, ADJUST PRECEDING ZEROES
        # ADJUST SPACING TO MATCH NEEDED FORMAT, THEN CREATE FINAL STRING FOR
        # OUTPUT/USAGE

        convM0 = str(m0)
        convM1 = str(m1)
        convM2 = str(m2)
        convM3 = str(m3)
        convM4 = str(m4)
        convM5 = str(m5)

        zeroFix = [convM0, convM1, convM2, convM3, convM4, convM5]

        for x in range(0, 6):
            temp = zeroFix[x]
            if (len(temp) == 1):
                temp = "00" + temp
            elif (len(temp) == 2):
                temp = "0" + temp

            zeroFix[x] = temp

        finalOut = zeroFix[0] + " " + zeroFix[1] + " " + zeroFix[2] + " " + zeroFix[3] + " " + zeroFix[4] + " " + zeroFix[5]

        try:

            data_to_send = finalOut + "\n"  # Include "\n" for end of line
            ser.write(data_to_send.encode('utf-8'))


        except KeyboardInterrupt:
            ser.close()

        self.get_logger().info(f'data sent to Arduino : {finalOut}')  # stop: 499 | max: 999(+), 000 (-) | move a little: 555 (+)

    def clip(self, value):
        min = -1
        max = 1

        if value < min:
            value = min
        if value > max:
            value = max

        return value


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
