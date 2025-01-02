import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from llm_action_interfaces.action import QueryEngine
from llm_action_interfaces.msg import Response, PartialResponse



class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('llm_action_client')
        self._action_client = ActionClient(self, QueryEngine, 'llm_action')
        self.declare_parameter('prompt',"Hello, my name is skippy. How can I help you today?")

    def send_goal(self, prompt):
        goal_msg = QueryEngine.Goal()
        goal_msg.prompt = self.get_parameter('prompt').get_parameter_value().string_value

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
        rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_response.text))
        
def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()
    prompt = action_client.get_parameter('prompt').get_parameter_value().string_value
    action_client.send_goal(prompt)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
