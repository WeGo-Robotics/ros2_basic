import rclpy
from rclpy.node import Node

# for tf listener
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# to control the turtle2
from geometry_msgs.msg import Twist
import math

class CarrotFollower(Node):
    def __init__(self):
        # set node initializer
        super().__init__('carrot_follower')

        # set the buffer for the listener
        self.tf_buffer = Buffer()

        # set listener and load data on tf_buffer
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # set publisher to control the turtle2 the publisher hz is 10
        self.pub = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # set the exception if there is no broadcast in 0.1 sec from now 
        # if exception thrown and print exception and finish the timer callback
        try:
            t = self.tf_buffer.lookup_transform(
                'turtle2',
                'carrot',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
        except TransformException as ex:
            print(ex)
            return
        
        # if transform is set properly make the contrl and publish        
        msg = Twist()

        # for the control data of angular velocity with heading of carrot from turtle2 
        msg.angular.z = 0.5 * math.atan2(
                    t.transform.translation.y,
                    t.transform.translation.x)

        # for the control data of linear velocity with distance of carrot from turtle2
        msg.linear.x = 0.5 * math.sqrt(
                    t.transform.translation.x ** 2 +
                    t.transform.translation.y ** 2)
        
        # publish the control data
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    carrot_follower = CarrotFollower()
    
    rclpy.spin(carrot_follower)
    carrot_follower.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()