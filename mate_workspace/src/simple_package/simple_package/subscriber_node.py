import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class SubscriberNode(Node):

    def __init__(self):
        super().__init__('subscriber_node') # run Node constructor
        self.subscription = self.create_subscription(
            String,
            'first_topic', # subscribe topic named topic
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # display message received
        self.get_logger().info('I heard: "%s"' % msg.data)


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