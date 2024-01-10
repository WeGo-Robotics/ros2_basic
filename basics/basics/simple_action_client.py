import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from wego_msgs.action import Timer


class SimpleActionClient(Node):

    def __init__(self):
        super().__init__('simple_action_client')
        self._action_client = ActionClient(self, Timer, 'timer')
    

    def send_goal(self, second):
        goal_msg = Timer.Goal()
        goal_msg.time_to_wait.sec = second

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self.goal_handle_ = goal_handle

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        print('time elapsed: ', result.time_elapsed.sec,'\nupdate count: ', result.updates_sent)
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.feedback = feedback_msg.feedback
        print('time elapsed: ', self.feedback.time_elapsed.sec, '\ntime_remaining: ', self.feedback.time_remaining.sec)
       


def main(args=None):
    rclpy.init(args=args)

    simple_action_client = SimpleActionClient()

    simple_action_client.send_goal(40)

    rclpy.spin(simple_action_client)


if __name__ == '__main__':
    main()