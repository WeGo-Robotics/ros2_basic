from wego_msgs.srv import WordCount

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')

        # client_cb_group = None
        # timer_cb_group = None
        
        client_cb_group = MutuallyExclusiveCallbackGroup()
        timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.cli = self.create_client(WordCount, 'word_count', callback_group=client_cb_group)
        self.timer_ = self.create_timer(1.0, self.timerCallback, callback_group=timer_cb_group)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.req = WordCount.Request()
        self.req.words = 'Hello world!'
        

    def timerCallback(self):
        print(f'Send data: {self.req.words}')
        word_count = self.cli.call(self.req)
        print(f'the number of word: {word_count.count}')

def main():
    rclpy.init()
    service_client = ServiceClient()

    executor = MultiThreadedExecutor()
    executor.add_node(service_client)
    executor.spin()

    service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()