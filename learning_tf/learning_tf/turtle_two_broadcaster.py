import math
from geometry_msgs.msg import TransformStamped
import numpy as np

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose
from turtlesim.srv import Spawn

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class TurtleTwoBoradcaster(Node):
    def __init__(self):
        super().__init__('turtle_two_broadcaster')
        self.turtlename = 'turtle2'

        self.cli = self.create_client(Spawn, 'spawn')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        req = Spawn.Request()
        req.x=1.0
        req.y=1.0
        req.name = self.turtlename
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.sub = self.create_subscription(
            Pose,
            f'/{self.turtlename}/pose',
            self.pose_callback,
            1)
        self.sub

    def pose_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtlename
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    turtle_two_broadcaster = TurtleTwoBoradcaster()
    
    rclpy.spin(turtle_two_broadcaster)
    turtle_two_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()