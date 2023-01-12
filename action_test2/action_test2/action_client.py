import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from action_test2_interfaces.action import ActionTest2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, QoSPresetProfiles
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

class ActionTest2Client(Node):
    def __init__(self):
        super().__init__('action_test2_client')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            #reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )
        self._action_client = ActionClient(
            self,
            ActionTest2,
            'action_test2',
            #callback_group=ReentrantCallbackGroup(),
            callback_group=MutuallyExclusiveCallbackGroup(),
            goal_service_qos_profile = qos_profile,
            result_service_qos_profile = qos_profile,
            cancel_service_qos_profile = qos_profile,
            feedback_sub_qos_profile = qos_profile,
            status_sub_qos_profile = qos_profile
        )

    def send_goal(self, status):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        goal_msg = ActionTest2.Goal()
        goal_msg.status = status
        #goal_msg.iparam = iparam
        #goal_msg.fparam = fparam
        self.get_logger().info('Sending goal request...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        while True:
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future)
            result = get_result_future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                return result
            self.get_logger().info('Get result failed :(')
            time.sleep(1.0)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))

def main(args=None):
    rclpy.init(args=args)
    action_client = ActionTest2Client()
    for count in range(1000000):
        action_client.get_logger().info('********************************************************'.format(count))
        goal_result = action_client.send_goal(1)
        result = goal_result.result
        status = goal_result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            action_client.get_logger().info('Goal succeeded! Result: {0}'.format(result))
        else:
            action_client.get_logger().info('Goal failed with status: {0}'.format(status))
            time.sleep(1)
        time.sleep(0.01)

if __name__ == '__main__':
    main()
