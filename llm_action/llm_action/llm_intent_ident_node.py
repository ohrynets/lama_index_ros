#!/usr/bin/env python3

# MIT License

# Copyright (c) 2023  Miguel Ángel González Santamarta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from llm_action_interfaces.action import QueryEngine
from turret_interfaces.msg import SlackMessage
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from llm_action.prompt.prompt_utils import PromptTemplateCache
from ament_index_python.packages import get_package_share_directory
import os

class LlmIntentIdentificationNode(Node):
    
    def prompt_received(self, msg:SlackMessage):
        self.get_logger().info(f"Prompt received ...{msg.text}")       
        self.send_prompt(msg)
    
    def __init__(self) -> None:
        super().__init__("llm_intent_ident_node")
        self.bridge = CvBridge()
        
        self.declare_parameter('input_topic', 'intent_in')
        self.declare_parameter('output_topic', 'intent_out')
        self.declare_parameter('llm_action_name', 'llm_action')     

        self.slack_input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.slack_output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.llm_action_name = self.get_parameter('llm_action_name').get_parameter_value().string_value

        custom_qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            deadline=Duration(seconds=1)
        )
        
        self.subscription = self.create_subscription(
            SlackMessage,
            self.slack_input_topic,
            self.prompt_received,
            10)
        
        self.publisher_ = self.create_publisher(SlackMessage, self.slack_output_topic, 10)

        self.tokens = 0
        self.initial_time = -1
        self.eval_time = -1

        self._action_client = ActionClient(self, QueryEngine, self.llm_action_name)                
        self.get_logger().info(f"Node started. Reading messages from: {self.slack_input_topic} ...")
        package_share_directory = os.path.join(get_package_share_directory('llm_action'), 'templates')
        self.template_cache = PromptTemplateCache(package_share_directory)
        self.get_logger().info(f"Template cache loaded from: {package_share_directory}")
        self.intent_identification_template = self.template_cache.get_template(
            'intent_identification'
        ) 
        
    def text_cb(self, feedback) -> None:

        if self.eval_time < 0:
            self.eval_time = time.time()

        self.tokens += 1
        print(feedback.feedback.partial_response.text, end="", flush=True)

    def send_prompt(self, prompt_msg:SlackMessage) -> None:

        goal_msg = QueryEngine.Goal()
        prompt = self.intent_identification_template.format(message=prompt_msg.text)
        
        goal_msg.prompt = prompt

        self.initial_time = time.time()
        if prompt_msg.is_screenshot:
            goal_msg.image = prompt_msg.screenshot
        self.get_logger().info(f"Received prompt: {prompt_msg.text} image:{prompt_msg.is_screenshot}")
        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
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
        self.get_logger().info('Result: {0}'.format(result.response.text))
        slack_msg = SlackMessage()
        slack_msg.text = result.response.text
        self.publisher_.publish(slack_msg)        

        self.get_logger().info("END")
        end_time = time.time()
        self.get_logger().info(
            f"Time to eval: {self.eval_time - self.initial_time} s")
        self.get_logger().info(
            f"Prediction speed: {self.tokens / (end_time - self.eval_time)} t/s")
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_response.text))
           
def main():
    rclpy.init()
    llama_node = LlmIntentIdentificationNode()
    rclpy.spin(llama_node)
    llama_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
