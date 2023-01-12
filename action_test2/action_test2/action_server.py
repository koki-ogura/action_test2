import time
import gc
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from action_test2_interfaces.action import ActionTest2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, QoSPresetProfiles
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

class ActionTest2Server(Node):
    def __init__(self):
        super().__init__('action_test2_server')
        self.feedback = ActionTest2.Feedback()
        self.result = ActionTest2.Result()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            #reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )
        self._action_server = ActionServer(
            self,
            ActionTest2,
            'action_test2',
            execute_callback=self.execute_callback,
            #callback_group=ReentrantCallbackGroup(),
            callback_group=MutuallyExclusiveCallbackGroup(),
            #goal_callback=self.goal_callback,
            #cancel_callback=self.cancel_callback,
            goal_service_qos_profile = qos_profile,
            result_service_qos_profile = qos_profile,
            cancel_service_qos_profile = qos_profile,
            #feedback_pub_qos_profile = qos_profile,
            feedback_pub_qos_profile = qos_profile,
            status_pub_qos_profile = qos_profile,
            result_timeout = 1
        )

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.get_logger().info('Receive goal request')
        del goal_request
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Receive cancel request')
        del goal_handle
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        status = 0
        #iparam = goal_handle.request.iparam
        #fparam = goal_handle.request.fparam
        for i in range(5):
            #if goal_handle.is_cancel_requested:
            #    goal_handle.canceled()
            #    self.get_logger().info('Goal canceled')
            #    return TMGripper.Result()
            status += 1
            #iparam += 1
            #fparam += 1.0
            #feedback = TMGripper.Feedback()
            self.feedback.status = status
            #self.feedback.iparam = iparam
            #self.feedback.fparam = fparam
            #self.get_logger().info('Publishing feedback: {0}'.format(feedback))
            goal_handle.publish_feedback(self.feedback)
            time.sleep(0.01)

        status += 1
        #iparam += 1
        #fparam += 1.0
        goal_handle.succeed()
        #result = TMGripper.Result()
        self.result.status = status
        #self.result.iparam = iparam
        #self.result.fparam = fparam
        #self.get_logger().info('Returning result: {0}'.format(result))
        #del goal_handle
        #gc.collect()
        time.sleep(0.01)
        return self.result

def main(args=None):
    rclpy.init(args=args)
    action_test2_server = ActionTest2Server()
    #executor = MultiThreadedExecutor()
    executor = SingleThreadedExecutor()
    rclpy.spin(action_test2_server, executor=executor)
    #rclpy.spin(tmgripper_action_server)
    #while rclpy.ok():
    #    rclpy.spin_once(tmgripper_action_server, timeout_sec=1.0)
    action_test2_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
