import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from wego_msgs.action import Timer

class FancyActionServer(Node):

    def __init__(self):
        super().__init__('fancy_action_server')
        self._action_server = ActionServer(
            self,
            Timer,
            'timer',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
    
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT


    async def execute_callback(self, goal_handle):
        start_time = time.time()
        update_count = 0
        
        result = Timer.Result()
        feedback = Timer.Feedback()

        if goal_handle.request.time_to_wait.sec > 60:  
            result.time_elapsed.sec = int(time.time()-start_time)
            result.updates_sent = update_count
            goal_handle.abort()
            return result

        while int(time.time() - start_time) < goal_handle.request.time_to_wait.sec:
            if goal_handle.is_cancel_requested:
                print('canceled requested')
                result.time_elapsed.sec = int(time.time() - start_time)
                result.updates_sent = update_count
                goal_handle.canceled()
                return result
            
            feedback.time_elapsed.sec = int(time.time() - start_time)
            feedback.time_remaining.sec = goal_handle.request.time_to_wait.sec - feedback.time_elapsed.sec
            goal_handle.publish_feedback(feedback)
            update_count +=1
            time.sleep(1.0)
        
        result.time_elapsed.sec = int(time.time() - start_time)
        result.updates_sent = update_count
        goal_handle.succeed()
        return result



def main(args=None):
    rclpy.init(args=args)

    fancy_action_server = FancyActionServer()

    executor = MultiThreadedExecutor()

    rclpy.spin(fancy_action_server,executor = executor)

    fancy_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()