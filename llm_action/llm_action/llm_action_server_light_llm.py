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

import rclpy
from llm_action.llm_action_server import QueryEngineActionServer

def main(args=None):
    rclpy.init(args=args)
    llama_index_action_server = QueryEngineActionServer(node_name= 'llm_light_action_node',
                                                        action_name='llm_action_light',
                                                        default_llm='llama3.2:3b')

    try:
        rclpy.spin(llama_index_action_server)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()