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
from llm_action.llm_tools import LamaIndexToRosTools
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from llama_index.llms.ollama import Ollama
from llama_index.multi_modal_llms.ollama import OllamaMultiModal
import nest_asyncio
from llama_index.core.llms import ChatMessage
from llama_index.core import PromptTemplate
from llama_index.core.multi_modal_llms import (
    MultiModalLLM,
    MultiModalLLMMetadata,
)
from llama_index.core.schema import ImageDocument, ImageNode
from cv_bridge import CvBridge
import cv2
import base64
import logging
import sys
from llama_index.core.callbacks import (
    CallbackManager,
    LlamaDebugHandler,
    CBEventType,
)
from llama_index.core import SimpleDirectoryReader
from llama_index.core.img_utils import img_2_b64
from PIL import Image
from typing import Any
from llama_index.core.base.llms.types import (
    ChatMessage,
    ChatResponse,
    ChatResponseAsyncGen,
    ChatResponseGen,
    CompletionResponse,
    CompletionResponseAsyncGen,
    CompletionResponseGen,
)
from typing import Any, Dict, List, Optional, Sequence, Tuple, get_args
from llama_index.core.multi_modal_llms.generic_utils import image_documents_to_base64

def has_method(obj, method_name):
    return hasattr(obj, method_name) and callable(getattr(obj, method_name))


def get_additional_kwargs(
    response: Dict[str, Any], exclude: Tuple[str, ...]
) -> Dict[str, Any]:
    if has_method(response, "items"):
        return {k: v for k, v in response.items() if k not in exclude}
    else:
        return {}

class FixedOllamaMultiModal(OllamaMultiModal):
    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        
    def complete(
        self,
        prompt: str,
        image_documents: Sequence[ImageNode],
        formatted: bool = False,
        **kwargs: Any,
    ) -> CompletionResponse:
        """Complete."""
        response = self._client.generate(
            model=self.model,
            prompt=prompt,
            images=image_documents_to_base64(image_documents),
            stream=False,
            options=self._model_kwargs,
            **kwargs,
        )
        return CompletionResponse(
            text=response["response"],
            raw=response,
            additional_kwargs=get_additional_kwargs(response, ("response",)),
        )
        
    def get_additional_kwargs(
        response: Dict[str, Any], exclude: Tuple[str, ...]
        ) -> Dict[str, Any]:
        return {k: v for k, v in response.items() if k not in exclude}
      
class QueryEngineActionServer(Node):

    def __init__(self):
        nest_asyncio.apply()
        super().__init__('llm_action_server')
        self._action_server = ActionServer(
            self,
            QueryEngine,
            'llm_action',
            self.execute_callback)
        self.tools = LamaIndexToRosTools()
        self.declare_parameter('ollama_url',"http://host.docker.internal:11434")
        self.ollama_url = self.get_parameter('ollama_url').get_parameter_value().string_value
        self.declare_parameter('ollama_model',"llama3.1:8b")
        self.ollama_model = self.get_parameter('ollama_model').get_parameter_value().string_value
        
        logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)
        logging.getLogger().addHandler(logging.StreamHandler(stream=sys.stdout))
        llama_debug = LlamaDebugHandler(print_trace_on_end=True)
        callback_manager = CallbackManager([llama_debug])
        self.llm: MultiModalLLM = FixedOllamaMultiModal(model=self.ollama_model,
                                                   request_timeout=120.0,
                                                   base_url=self.ollama_url, 
                                                   verbose=True,
                                                   num_ctx=1024,)
        self.get_logger().info(f"LLM Action Server started with model: {self.ollama_model} and url: {self.ollama_url}")
        #self.llm = Ollama(model=self.ollama_model, request_timeout=120.0, base_url=self.ollama_url, verbose=True)

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Executing goal on {self.ollama_url}...")
        input_msg = [ChatMessage.from_str(goal_handle.request.prompt)]
        feedback_msg = QueryEngine.Feedback()
        feedback_msg.partial_response = PartialResponse()
        feedback_msg.partial_response.text = "" 
        self.get_logger().info(f'Goal:{goal_handle.request.prompt}')        
        if goal_handle.request.image and goal_handle.request.image.data:
            self.get_logger().info(f'Prompt:{goal_handle.request.prompt} with an image')
            image_nodes = self.tools.ros2_to_image_document(goal_handle.request.image)
            # opencv_img = self.bridge.imgmsg_to_cv2(goal_handle.request.image, "bgr8")
            # # Convert from BGR (OpenCV) to RGB (PIL) color space
            # rgb_image = cv2.cvtColor(opencv_img, cv2.COLOR_BGR2RGB)

            # # Convert to PIL Image
            # pil_image = Image.fromarray(rgb_image)
            # b64_img = img_2_b64(pil_image)
            # image_nodes = [ImageDocument(image=b64_img, mimetype="jpeg")]
            output = self.llm.complete(prompt=goal_handle.request.prompt, image_documents=image_nodes)
        else:
            self.get_logger().info(f'Prompt:{goal_handle.request.prompt}')
            output = self.llm.complete(prompt=goal_handle.request.prompt, image_documents=[])
            
        # for partial_output in stream_output:
        #     feedback_msg.partial_response.text=feedback_msg.partial_response.text + partial_output.delta
        #     self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_response))
        #     goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(f'Response:{output.text}')
        goal_handle.succeed()

        result = QueryEngine.Result()
        result.response
        result.response.text = output.text
        return result


def main(args=None):
    rclpy.init(args=args)
    llama_index_action_server = QueryEngineActionServer()

    try:
        rclpy.spin(llama_index_action_server)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()