import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from rio_interfaces.action import TTS
import ollama
import re

class OllamaNLPNode(Node):
    def __init__(self):
        super().__init__('ollama_nlp_node')
        self.subscription = self.create_subscription(
            String,
            '/speech_recognition/result',
            self.listener_callback,
            10)
        
        self.tts_action_client = ActionClient(self, TTS, '/tts')
        self.expression_map = {
            'idle': 0, 'speaking': 1, 'curious': 2, 'afraid': 3,
            'blush': 4, 'angry': 5, 'sad': 6, 'happy': 7,
            'surprise': 8, 'sleep': 9, 'wakeup': 10, 'listening': 11,
            'thinking': 12
        }
        
        # Load parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_name', 'llama3'),  # Default if not specified
                ('system_prompt', ''),
                ('max_history_length', 5)  # Keep last 5 exchanges
            ]
        )
        
        # Get parameters
        self.model_name = self.get_parameter('model_name').value
        self.system_prompt = self.get_parameter('system_prompt').value
        
        # Add conversation history
        self.message_history = []
        
        # Initialize with system prompt
        self.message_history.append({
            'role': 'system',
            'content': self.system_prompt
        })
        
        self.get_logger().info(f"Using Ollama model: {self.model_name}")

    def listener_callback(self, msg):
        user_input = msg.data
        self.get_logger().info(f"Received query: {user_input}")
        
        # Add user message to history
        self.message_history.append({'role': 'user', 'content': user_input})
        
        # Get Ollama response with full history
        response = ollama.chat(
            model=self.model_name,
            messages=self._get_truncated_history()
        )
        
        # Process response
        full_response = response['message']['content']
        
        # Add assistant response to history
        self.message_history.append({
            'role': 'assistant',
            'content': full_response
        })
        
        # Parse response
        match = re.match(r'^\s*\[([^\]]+)\](.+)$', full_response, re.DOTALL)
        if match:
            expression = match.group(1).strip().lower()
            response_text = match.group(2).strip()
        else:
            # If format not followed, use entire response as text with default expression
            expression = 'idle'
            response_text = full_response.strip()
            self.get_logger().warn("Response format invalid, using default expression")

        # Clean up text
                # Clean up text - enhanced emoji removal
        response_text = re.sub(r'[^\w\s,.!?\-()\'"]', '', response_text, flags=re.UNICODE)  # Remove all non-word chars
        response_text = re.sub(r'\s+', ' ', response_text).strip()  # Clean whitespace
        
        # Final validation
        if not response_text:
            response_text = "Let me think about that"
            
        if expression not in self.expression_map:
            expression = 'idle'

        self.get_logger().info(f"Parsed expression: {expression}")
        self.get_logger().info(f"Final response text: {response_text}")
        
        # Send to TTS
        self.send_tts_goal(response_text, expression)

    def send_tts_goal(self, text, expression):
        goal_msg = TTS.Goal()
        goal_msg.text = text
        goal_msg.voice_output = True
        goal_msg.start_expression = expression
        goal_msg.end_expression = 'idle'
        goal_msg.expression_sound = False

        self.tts_action_client.wait_for_server()
        self._send_goal_future = self.tts_action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'TTS completed with: {result.success}')

    def _get_truncated_history(self):
        """Keep only the most recent messages within history limit"""
        max_length = self.get_parameter('max_history_length').value * 2 + 1  # Account for system prompt
        return self.message_history[-max_length:]

def main(args=None):
    rclpy.init(args=args)
    node = OllamaNLPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()