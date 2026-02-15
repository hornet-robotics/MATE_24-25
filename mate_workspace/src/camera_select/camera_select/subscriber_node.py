import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SubscriberNode(Node):

    def __init__(self):
        super().__init__('subscriber_node') # run Node constructor
        self.subscription = self.create_subscription(
            String,
            'selected_camera', # subscribe topic named topic
            self.camera_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Camera Subscriber Node Initialized")

    def listener_callback(self, msg):
        selected_camera = msg.data
        self.get_logger().info(f"Camera selected: {selected_camera}")
        # Add any code here to switch the camera feed or perform other actions


def main(args=None):
    rclpy.init(args=args)

    # create subscriber node obj
    subscriber_node = SubscriberNode()

    try:
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        pass
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
