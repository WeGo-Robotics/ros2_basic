import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ParameterExample(Node):
    def __init__(self):
        super().__init__('parameter_example')
        self.declare_parameter('message_content', 'Default message')
        self.publisher_ = self.create_publisher(String,
                                                'example_topic',
                                                 10)
        self.timer_ = self.create_timer(0.5, self.check_parameter_and_publish)

    def check_parameter_and_publish(self):
        current_message = self.get_parameter('message_content').value
        msg = String()
        msg.data = current_message
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ParameterExample()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
