import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard


class CameraSelectNode(Node):

    def __init__(self):
        super().__init__('camera_select') # run Node constructor
        self.camera_publisher = self.create_publisher(String, 'selected_camera', 10) # publish to topic named select_camera 
        self.get_logger().info("Camera Select Node Initialized. Press 1,2,3 to select.")

        #Start keyboard listener
        listener = keyboard.listener(on_press=self.on_key_press)
        listener.start()

    def on_key_press(self, key):
        try:
            #Check for key presses
            if key.char == '1'
                self.select_camera('camera1')
            elif key.char == '2'
                self.select_camera('camera2')
            elif key.char == '3'
                self.select_camera('camera3')
        except AttributeError:
            pass

    def select_camera(self, camera_name):
        msg = String()
        msg.data = camera_name
        self.camera_publisher.publish(msg)
        self.get_logger().info(f"Selected {camera_name}")


def main(args=None):
    rclpy.init(args=args)

    # create publisher node obj
    publisher_node = CameraSelectNode()

    # run Node
    try:
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        pass
    publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
