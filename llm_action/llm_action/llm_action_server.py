#!/usr/bin/env python3
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.import time

import time

from llm_action_interfaces.action import QueryEngine
from llm_action_interfaces.msg import Response, PartialResponse

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from llama_index.llms.ollama import Ollama
import nest_asyncio
from llama_index.core.llms import ChatMessage
from llama_index.core import PromptTemplate


class QueryEngineActionServer(Node):

    def __init__(self):
        nest_asyncio.apply()
        super().__init__('llm_action_server')
        self._action_server = ActionServer(
            self,
            QueryEngine,
            'llm_action',
            self.execute_callback)
        self.declare_parameter('ollama_url',"http://host.docker.internal:11434")
        self.ollama_url = self.get_parameter('ollama_url').get_parameter_value().string_value
        self.declare_parameter('ollama_model',"llama3.1:8b")
        self.ollama_model = self.get_parameter('ollama_model').get_parameter_value().string_value
        
        self.llm = Ollama(model=self.ollama_model, request_timeout=120.0, base_url=self.ollama_url, 
                is_function_calling_model=True)
        

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Executing goal on {self.ollama_url}...")
        input_msg = ChatMessage.from_str(goal_handle.request.prompt)
        prompt_template = PromptTemplate("{message}")
        self.llm.predict(prompt_template, message=goal_handle.request.prompt)
        feedback_msg = QueryEngine.Feedback()
        feedback_msg.partial_response = PartialResponse()
        feedback_msg.partial_response.text = "" 
        self.get_logger().info(f'Goal:{goal_handle.request.prompt}')
        input_msg = ChatMessage.from_str(goal_handle.request.prompt)
        stream_output = self.llm.stream_chat([input_msg])
        for partial_output in stream_output:
            feedback_msg.partial_response.text=feedback_msg.partial_response.text + partial_output.delta
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_response))
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        result = QueryEngine.Result()
        result.response
        result.response.text = feedback_msg.partial_response.text
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = QueryEngineActionServer()

    try:
        rclpy.spin(fibonacci_action_server)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()