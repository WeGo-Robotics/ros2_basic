import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LaunchExample(Node):
    def __init__(self):
        super().__init__('launch_example')

        self.declare_parameter('topic_name', 'default_topic') 
        topic_name = self.get_parameter('topic_name')
        self.publisher_ = self.create_publisher(String, topic_name.value, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'this is launch example topic'
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    launch_example = LaunchExample()
    rclpy.spin(launch_example)
    launch_example.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()