import time

import rclpy
from rclpy.action import ActionServer

from rclpy.node import Node

from wego_msgs.action import Timer

class SimpleActionServer(Node):

    def __init__(self):
        super().__init__('simple_action_server')
        self._action_server = ActionServer(
            self,
            Timer,
            'timer',
            execute_callback=self.execute_callback)
    
    def execute_callback(self, goal_handle):
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

    simple_action_server = SimpleActionServer()

    rclpy.spin(simple_action_server)


if __name__ == '__main__':
    main()