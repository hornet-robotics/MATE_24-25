import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


import serial
import time

arduino_port = '/dev/ttyACM0'
baud_rate = 9600

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

        #ADDING SEGMENT TO CONVERT M0-M5 TO STRING, ADJUST PRECEDING ZEROES
        #ADJUST SPACING TO MATCH NEEDED FORMAT, THEN CREATE FINAL STRING FOR
        #OUTPUT/USAGE

        convM0 = str(m0)
        convM1 = str(m1)
        convM2 = str(m2)
        convM3 = str(m3)
        convM4 = str(m4)
        convM5 = str(m5)

        zeroFix = [convM0, convM1, convM2, convM3, convM4, convM5]

        for x in range(0, 5):
            temp = zeroFix[x]
            if (len(temp) == 1) :
                temp = "00" + temp
            elif (len(temp) == 2) :
                temp = "0" + temp

            zeroFix[x] = temp

        finalOut = zeroFix[0] + " " + zeroFix[1] + " " + zeroFix[2] + " " + zeroFix[3] + " " + zeroFix[4] + " " + zeroFix[5]

        try:
            ser = serial.Serial(arduino_port, baud_rate)
            #time.sleep(2)  # Allow time for the serial connection to initialize

            data_to_send = finalOut + "\n"  # Include "\n" for end of line
            ser.write(data_to_send.encode('utf-8'))
            # print(f"Sent: {data_to_send}")

            #ser.close()

        except serial.SerialException as e:
            pass
          # print(f"Error: {e}")

        self.get_logger().info(f'From joystick_topic I heard : {msg.data} | data sent to Arduino : {finalOut}')


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
