import rclpy
from rclpy.node import Node

# turtlesim msg to subscribe the turtle pose
from turtlesim.msg import Pose

# to broadcast tf
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# to convert euler from quaternion
from wego_tf.quaternion_from_euler import quaternion_from_euler


class TurtleOneBroadcast(Node):
    def __init__(self):
        # node initializer
        super().__init__('turtle_one_broadcaster')
        
        # set tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # set subscriber for turtle1/pose
        self.sub_ = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            1
        )
        # just for prevent from warning unused variable
        self.sub_
    
    def pose_callback(self, msg):
        # to transform the from of the data
        # must be geometry_msgs.msg TransformStamped
        # just fill in the form
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg() 
        t.header.frame_id = 'world'
        t.child_frame_id = 'turtle1'
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # get quaternion from euler value
        q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # broad cast
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    turtle_one_broadcast = TurtleOneBroadcast()

    rclpy.spin(turtle_one_broadcast)

    turtle_one_broadcast.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__':
    main()