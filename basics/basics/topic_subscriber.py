import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32


class TopicSubscriber(Node):

    def __init__(self):
        super().__init__('topic_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'counter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        print(msg.data)


def main(args=None):
    rclpy.init(args=args)

    topic_subscriber = TopicSubscriber()

    rclpy.spin(topic_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    topic_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()