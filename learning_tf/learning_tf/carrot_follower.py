import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class CarrotFollower(Node):
    def __init__(self):
        super().__init__('carrot_follower')
        self.turtlename = 'turtle2'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)
        self.timer = self.create_timer(0.1, self.timer_callback)


    def timer_callback(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'turtle2',
                'carrot',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
        except TransformException as ex:
            print(ex)
            return
        
        msg = Twist()
        msg.angular.z = 0.5 * math.atan2(
                    t.transform.translation.y,
                    t.transform.translation.x)

        msg.linear.x = 0.5 * math.sqrt(
                    t.transform.translation.x ** 2 +
                    t.transform.translation.y ** 2)
        
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    carrot_follower = CarrotFollower()
    
    rclpy.spin(carrot_follower)
    carrot_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()