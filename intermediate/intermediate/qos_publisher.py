import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Int32

class QOSPublisher(Node):
    def __init__(self):
        super().__init__('qos_publisher')

        self.default_pub_ = self.create_publisher(Int32, 'default_topic', 10)
        self.sensor_pub_ = self.create_publisher(Int32,
                                                'sensor_topic', 
                                                rclpy.qos.qos_profile_sensor_data)
        
        custom_profile = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.VOLATILE,
            history = HistoryPolicy.KEEP_LAST,
            depth = 10
        )

        self.custom_pub_ = self.create_publisher(Int32, 'custom_topic', custom_profile)

        self.timer_ = self.create_timer(1.0, self.timerCallback)
        self.count = 0

    def timerCallback(self):
        msg = Int32()
        msg.data = self.count

        self.default_pub_.publish(msg)
        self.sensor_pub_.publish(msg)
        self.custom_pub_.publish(msg)

        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    qospub = QOSPublisher()
    
    rclpy.spin(qospub)

    qospub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
