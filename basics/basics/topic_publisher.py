import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32


class TopicPublisher(Node):

    def __init__(self):
        super().__init__('topic_publisher')
        self.publisher_ = self.create_publisher(Int32, 'counter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = Int32()
        msg.data = self.count
        self.publisher_.publish(msg)
        self.count += 1


def main(args=None):
    rclpy.init(args=args)

    topic_publisher = TopicPublisher()

    rclpy.spin(topic_publisher)

    topic_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()