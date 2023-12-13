from wego_msgs.srv import WordCount

import rclpy
from rclpy.node import Node


class ServiceServer(Node):

    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(WordCount, 'word_count', self.count_words)

    def count_words(self, request, response):
        response.count = len(request.words.split())

        return response


def main():
    rclpy.init()

    service_server = ServiceServer()

    rclpy.spin(service_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()