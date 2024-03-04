import rclpy
from rclpy.node import Node

# turtlesim msg to subscribe the turtle pose
from turtlesim.msg import Pose

# to spawn the turtle2
from turtlesim.srv import Spawn

# to broadcast tf
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# to convert euler from quaternion
from wego_tf.quaternion_from_euler import quaternion_from_euler


class TurtleTwoBroadcast(Node):
    def __init__(self):
        # node initializer
        super().__init__('turtle_two_broadcaster')

        # spawn turtle2
        self.cli = self.create_client(Spawn, 'spawn')
        
        # wait for service server
        while not self.cli.wait_for_service(timeout_sec=1.0):
            print('service not available, waiting for turtlesim') 

        # send request to server to spawn
        req = Spawn.Request()
        req.name = 'turtle2'
        future = self.cli.call_async(req)
        
        # wait until turtle2 to spawn
        rclpy.spin_until_future_complete(self, future)
               
        # set tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # set subscriber for turtle2/pose
        self.sub_ = self.create_subscription(
            Pose,
            '/turtle2/pose',
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
        t.child_frame_id = 'turtle2'
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
    turtle_two_broadcast = TurtleTwoBroadcast()

    rclpy.spin(turtle_two_broadcast)

    turtle_two_broadcast.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__':
    main()