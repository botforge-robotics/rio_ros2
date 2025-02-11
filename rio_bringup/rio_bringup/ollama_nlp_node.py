import rclpy
import re
import ollama
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rio_interfaces.action import TTS
from ament_index_python.packages import get_package_share_directory
import os
from action_msgs.msg import GoalStatus

class OllamaNLPNode(Node):
    def __init__(self):
        super().__init__('ollama_nlp_node')
        
        # ROS2 Setup
        self.subscription = self.create_subscription(
            String, '/speech_recognition/result', self.listener_callback, 10)
        self.tts_action_client = ActionClient(self, TTS, '/tts')
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Expression mapping
        self.expression_map = {
            'idle': 0, 'speaking': 1, 'curious': 2, 'afraid': 3,
            'blush': 4, 'angry': 5, 'sad': 6, 'happy': 7,
            'surprise': 8, 'sleep': 9, 'wakeup': 10, 'listening': 11,
            'thinking': 12
        }
        
        # Declare parameter for config file path
        self.declare_parameter('navigation_locations_path', 
                              os.path.join(
                                  get_package_share_directory('rio_bringup'),
                                  'config',
                                  'navigation_locations.yaml'
                              ))
        
        # Load navigation locations
        self.nav_locations = self._load_navigation_locations()
        
        # Declare parameters without type specification
        self.declare_parameter('model_name', 'llama3')
        self.declare_parameter('system_prompt', '')
        self.declare_parameter('max_history_length', 5)
        
        # Get parameter values
        self.model_name = self.get_parameter('model_name').value
        self.system_prompt = self.get_parameter('system_prompt').value
        self.max_history_length = self.get_parameter('max_history_length').value
        
        # Initialize components
        self.message_history = []
        self._initialize_system_prompt()

   

    def _load_navigation_locations(self):
        """Load navigation locations directly from YAML file"""
        config_path = self.get_parameter('navigation_locations_path').value
        
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                locations = config.get('locations', {})
                
                if not locations:
                    self.get_logger().warn("No locations found in config file")
                
                self.get_logger().info(f"Loaded {len(locations)} locations")
                return locations
                
        except FileNotFoundError:
            self.get_logger().error(f"Config file not found: {config_path}")
            return {}
        except yaml.YAMLError as e:
            self.get_logger().error(f"Invalid YAML in config file: {str(e)}")
            return {}
        except Exception as e:
            self.get_logger().error(f"Error loading config: {str(e)}")
            return {}

    def _initialize_system_prompt(self):
        """Format system prompt with actual locations and expressions"""
        location_list = ", ".join(self.nav_locations.keys()) if self.nav_locations else "None configured"
        expressions_list = "speaking, curious, afraid, blush, angry, sad, happy, surprise"
        
        self.system_prompt = self.system_prompt.format(
            locations=location_list,
            expressions_list=expressions_list
        )
        self.message_history.append({'role': 'system', 'content': self.system_prompt})

    def listener_callback(self, msg):
        """Process user input and generate response"""
        user_input = msg.data.strip()
        if not user_input:
            return
            
        self.get_logger().info(f"Processing query: {user_input}")
        
        # Update conversation history
        self._update_history('user', user_input)
        
        # Get AI response
        response = self._get_ai_response()
        self._update_history('assistant', response)
        
        # Process and execute response
        self._process_ai_response(response)

    def _get_ai_response(self):
        """Get response from Ollama with history context"""
        try:
            response = ollama.chat(
                model=self.model_name,
                messages=self._get_truncated_history()
            )
            return response['message']['content']
        except Exception as e:
            self.get_logger().error(f"Ollama error: {str(e)}")
            return "[sad] I'm having trouble thinking right now"

    def _process_ai_response(self, response):
        """Parse and execute the AI's response"""
        self.get_logger().info(f"Raw AI Response: '{response}'")
        
        # Check each line separately for commands
        for line in response.split('\n'):
            line = line.strip()
            # Handle navigation commands (with bracketed location and optional text)
            nav_match = re.match(r'^\[navigate\]\s+\[(\w+)\]\s+\[(\w+)\](?:\s+(.*))?$', line)
            if nav_match:
                expression, location, response_text = nav_match.groups()
                self._handle_navigation_command(nav_match)
                return  # Process only first valid command
                
        # Handle normal responses if no command found
        expr_match = re.search(r'^\[(\w+)\](.+)', response, re.DOTALL)
        if expr_match:
            expression, text = expr_match.groups()
        else:
            expression, text = 'idle', response
            
        self._send_tts(text.strip(), expression.lower())

    def _handle_navigation_command(self, match):
        """Execute navigation command"""
        expression = match.group(1).lower()
        location = match.group(2).lower()
        response_text = match.group(3)  # Capture the response text

        self.get_logger().info(f"Attempting navigation to: {location}")  # Debug 4
        
        if location not in self.nav_locations:
            self.get_logger().error(f"Unknown location: {location}")  # Debug 5
            self._send_tts(f"I don't know where {location} is", 'sad')
            return
            
        coords = self.nav_locations[location]
        if self._validate_coordinates(coords):
            self.get_logger().info(f"Sending goal to {location} with coords: {coords}")  # Debug 6
            self._send_navigation_goal(coords, location)
            # Use the response text from the AI's response
            if response_text:
                self._send_tts(response_text.strip(), expression)
            else:
                self._send_tts(f"Going to {location.replace('_', ' ')}", expression)
        else:
            self.get_logger().error(f"Invalid coordinates for {location}: {coords}")  # Debug 7

    def _validate_coordinates(self, coords):
        """Check coordinate validity"""
        required = ['x', 'y', 'theta']
        return all(key in coords and isinstance(coords[key], (int, float)) for key in required)

    def _send_navigation_goal(self, coords, location):
        """Send goal to navigation stack"""
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(coords['x'])
        pose.pose.position.y = float(coords['y'])
        pose.pose.orientation.z = float(coords['theta'])
        goal_msg.pose = pose
        
        self.nav_action_client.wait_for_server()
        self._nav_goal_future = self.nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback
        )
        self._nav_goal_future.add_done_callback(self._nav_response)

    def _nav_response(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected")
            return
            
        self._nav_result_future = goal_handle.get_result_async()
        self._nav_result_future.add_done_callback(self._nav_result)

    def _nav_result(self, future):
        """Handle navigation result"""
        result = future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._send_tts("Arrived at destination!", 'happy')
        # else:
        #     self._send_tts("Navigation failed", 'sad')

    def _nav_feedback(self, feedback):
        """Handle navigation progress updates"""
        remaining = feedback.feedback.distance_remaining
        # if remaining < 1.0:
        #     self._send_tts("Almost there!", 'curious')

    def _send_tts(self, text, expression):
        """Send text to TTS system with expression"""
        if expression not in self.expression_map:
            expression = 'idle'
            
        # Clean text
        clean_text = re.sub(r'[^\w\s,.!?\-\']', '', text, flags=re.UNICODE)
        clean_text = re.sub(r'\s+', ' ', clean_text).strip()
        
        if not clean_text:
            clean_text = "Let me think about that"
            
        goal_msg = TTS.Goal()
        goal_msg.text = clean_text
        goal_msg.voice_output = True
        goal_msg.start_expression = expression
        goal_msg.end_expression = 'idle'
        
        self.tts_action_client.wait_for_server()
        self._tts_goal_future = self.tts_action_client.send_goal_async(goal_msg)

    def _update_history(self, role, content):
        """Manage conversation history with rolling window"""
        max_length = self.max_history_length * 2 + 1
        self.message_history.append({'role': role, 'content': content})
        self.message_history = self.message_history[-max_length:]

    def _get_truncated_history(self):
        """Get recent history within configured limits"""
        max_length = self.max_history_length * 2 + 1
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