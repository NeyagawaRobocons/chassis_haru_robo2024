#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from mecha_control.action import DaizaCmd, HinaCmd
import time

class DaizaCmdActionClient(Node):
    def __init__(self):
        super().__init__('daiza_cmd_action_client')
        self._action_client = ActionClient(self, DaizaCmd, 'daiza_cmd')
        self.is_get_result = False

    def send_goal(self, command):
        self.is_get_result = False
        goal_msg = DaizaCmd.Goal()
        goal_msg.command = command

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.result}')
        self.is_get_result = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.feedback}')

class HinaCmdActionClient(Node):
    def __init__(self):
        super().__init__('hina_cmd_action_client')
        self._action_client = ActionClient(self, HinaCmd, 'hina_cmd')
        self.is_get_result = False

    def send_goal(self, command):
        self.is_get_result = False
        goal_msg = HinaCmd.Goal()
        goal_msg.command = command

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.result}')
        self.is_get_result = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.feedback}')

def main() -> None:
    rclpy.init()
    node = DaizaCmdActionClient()
    # node = HinaCmdActionClient()
    commands = [
        DaizaCmd.Goal.READY,
        DaizaCmd.Goal.EXPAND_AND_UNCLAMP,
        DaizaCmd.Goal.CLAMP_AND_CONTRACT,
        DaizaCmd.Goal.EXPAND_AND_PLACE_AND_CONTRACT,
        DaizaCmd.Goal.READY,
        # HinaCmd.Goal.READY,
        # HinaCmd.Goal.DOWN_AND_TAKE,
        # HinaCmd.Goal.UP_AND_CARRY,
        # HinaCmd.Goal.UP_AND_PLACE,
        # HinaCmd.Goal.LATCH_UNLOCK,
        # HinaCmd.Goal.READY,
        # HinaCmd.Goal.LATCH_UNLOCK_1,
        # HinaCmd.Goal.LATCH_UNLOCK_2,
        # HinaCmd.Goal.READY,
    ]
    for command in commands:
        node.send_goal(command)
        print(f"sending goal: {command}")
        rclpy.spin_until_future_complete(node, node._send_goal_future)
        print(f"result: {node._send_goal_future.result()}")
        time.sleep(1)
        print("counting down... 6")
        time.sleep(1)
        print("counting down... 5")
        time.sleep(1)
        print("counting down... 4")
        time.sleep(1)
        print("counting down... 3")
        time.sleep(1)
        print("counting down... 2")
        time.sleep(1)
        print("counting down... 1")
        time.sleep(1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()