import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Int32

class QOSSubscriber(Node):
    def __init__(self):
        super().__init__('qos_subscriber')

        self.default_sub_ = self.create_subscription(
                        Int32,
                        'default_topic',
                        self.defaultCallback,
                        10
                    )
        
        self.sensor_sub_ = self.create_subscription(
                            Int32,
                            'sensor_topic',
                            self.sensorCallback,
                            rclpy.qos.qos_profile_sensor_data
                        )
        
        custom_profile = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.VOLATILE,
            history = HistoryPolicy.KEEP_LAST,
            depth = 10
        )
        
        self.custom_sub_ = self.create_subscription(
            Int32,
            'custom_topic',
            self.customCallback,
            10 #custom_profile
        )
    
    def defaultCallback(self, msg):
        print(f'default_topic: {msg.data}')
    
    def sensorCallback(self, msg):
        print(f'sensor_topic: {msg.data}')
    
    def customCallback(self, msg):
        print(f'custom_topic: {msg.data}')
    
def main(args=None):
    rclpy.init(args=args)
    qos_sub = QOSSubscriber()

    rclpy.spin(qos_sub)

    qos_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()