import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
# from wego_msgs.msg import Counter

class TopicSubscriber(Node):
    def __init__(self):
        super().__init__('topic_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'counter',
            self.listener_callback,
            10)
        
        # self.subscription = self.create_subscription(
        #     Counter,
        #     'counter',
        #     self.listener_callback,
        #     10)

        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        print(msg.data)
        # print(msg.count)

def main(args=None):
    rclpy.init(args=args)

    topic_subscriber = TopicSubscriber()

    rclpy.spin(topic_subscriber)

    topic_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()