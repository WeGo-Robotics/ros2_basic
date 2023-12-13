import sys

from wego_msgs.srv import WordCount
import rclpy
from rclpy.node import Node


class ServiceClient(Node):

    def __init__(self):
        super().__init__('service_client')
        self.cli = self.create_client(WordCount, 'word_count')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = WordCount.Request()

    def send_request(self, a):
        self.req.words = a
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    service_client = ServiceClient()
    words = ' '.join(sys.argv[1:])

    response = service_client.send_request(words)
    print(words, '->', response.count)

    service_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()