import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

class ExecutorExample(Node):
    def __init__(self):
        super().__init__('executor_example')

        self.timer_ = self.create_timer(1.0, self.timerCallback)
    
    def timerCallback(self):
        print('callback called')

def main(args=None):
    rclpy.init(args=args)
    executor_example_node = ExecutorExample()

    #rclpy.spin()
    executor = SingleThreadedExecutor()
    executor.add_node(executor_example_node)
    executor.spin()

    executor_example.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()