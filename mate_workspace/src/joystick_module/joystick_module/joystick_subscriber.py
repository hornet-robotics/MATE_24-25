import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class SubscriberNode(Node):

    def __init__(self):
        super().__init__('joystick_subscriber') # run Node constructor
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'joystick_topic', # subscribe topic named topic
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # display message received
        self.get_logger().info(f'Received: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    # create subscriber node obj
    subscriber_node = SubscriberNode()

    # run Node
    rclpy.spin(subscriber_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()