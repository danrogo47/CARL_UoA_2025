"""
PS4 controller node.

Establishes the client-server connection, gets controller inputs,
and publishes ROS topics based on those inputs.
"""
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16, Float32
import time
from geometry_msgs.msg import Twist # Not necessary for CARL
import json
import re_rassor_controller.lib.controller_input_defs as inputs

from custom_msgs.msg import Joint # Import where necessary!!

class ControllerCommandPublisher(Node):
    """Receives controller commands, converts them to topics and publishes them."""

    def __init__(self):

        super().__init__('controller_command_publisher')
        # ROS topics to publish from the controller inputs        
        self.controller_state_publisher_ = self.create_publisher(String, 'controller_state', 100)

        self.velocity_publisher_ = self.create_publisher(Twist, 'cmd_vel', 100)
        self.speed_mode_publisher_ = self.create_publisher(Float32, 'speed_mode', 10)
        self.drive_mode_publisher_ = self.create_publisher(String, 'drive_mode', 10)
        self.wheel_selection_publisher_ = self.create_publisher(String, 'wheg_selection', 10)

        # set default speed multiplier to 25%
        self.prev_speed_multiplier = 0.25

        # set debounce time for button presses
        self.debounce_time = 0.5 # seconds
        self.circle_last_pressed_time = 0 
        self.square_last_pressed_time = 0
        self.cross_last_pressed_time = 0
        self.triange_last_pressed_time = 0
        self.ps_last_pressed_time = 0

        # speed mode message
        self.speed_mode_msg = Float32()
        self.speed_mode_msg.data = 0.10

        # drive mode message
        self.drive_mode_msg = String()
        self.drive_mode_msg.data = 'STANDARD'

        # Joint message
        self.joint_msg = Joint()
        self.joint_msg.t_joint.data = 'FRONT'

        # Wheg selection message
        self.wheel_msg = String()
        self.wheel_msg.data = 'FRONT'

        # set flags for buttons pressed
        self.circle_button_pressed = False
        self.cross_button_pressed = False
        self.square_button_pressed = False
        self.triangle_button_pressed = False

        self.receive_data()

    def receive_data(self):
        # Set the IP address and port for the server
        server_ip = '0.0.0.0'  # Listen on all available network interfaces
        server_port = 8000  # Choose a port number that is not in use

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.bind((server_ip, server_port))
            server_socket.listen(1)  # Listen for incoming connections
            self.get_logger().info('Server listening on port 8000...')

            while True:  # Keep the server running to accept multiple connections
                try:
                    client_socket, addr = server_socket.accept()
                    self.get_logger().info(f'Connection established with {addr}')
                    buffer = b""

                    with client_socket:
                        while True:  # Continuously receive data from the client
                            chunk = client_socket.recv(1024)
                            if not chunk:
                                break
                            buffer += chunk

                            # Process the buffer for complete JSON strings
                            while b'\n' in buffer:
                                msg_data, buffer = self.extract_json(buffer)
                                if msg_data:

                                    controller_msg = String()
                                    controller_msg.data = msg_data
                                    self.controller_state_publisher_.publish(controller_msg)
                                    
                                    data_array = json.loads(msg_data)

                                    # convert raw json strings to meaningful commands
                                    self.get_driving_commands(data_array)
                                    self.get_t_joint_commands(data_array)
                                    self.get_tool_commands(data_array)

                except socket.error as e:
                    self.get_logger().error(f'Socket error: {e}')
                except Exception as e:
                    self.get_logger().error(f'Unexpected error: {e}')

    def extract_json(self, buffer):
        """Extracts and returns a complete JSON string from the buffer."""
        parts = buffer.split(b'\n', 1)
        if len(parts) > 1:
            complete_json = parts[0].decode('utf-8').strip()
            remaining_buffer = parts[1]
            return complete_json, remaining_buffer
        else:
            return None, buffer
        
    def get_driving_commands(self, data):
        """Process and publish commands for driving."""

        # set the speed multiplier for driving the wheels
        if data['buttons'][inputs.SHARE] == 1:
            self.speed_mode_msg.data = 0.10
        elif data['buttons'][inputs.TOUCH_PAD] == 1:
            self.speed_mode_msg.data = 0.20
        elif data['buttons'][inputs.OPTIONS] == 1:
            self.speed_mode_msg.data = 0.30

        # set time counter
        current_time = time.time()
        debounce_time = 0.5 # seconds

        # Toggle the drive mode between standard and independent
        if (data['buttons'][inputs.PS] == 1) and (current_time - self.ps_last_pressed_time > debounce_time):

            self.ps_last_pressed_time = current_time

            # toggle the interchange
            if self.drive_mode_msg.data == 'STANDARD':
                self.drive_mode_msg.data = 'INDEPENDENT'
            elif self.drive_mode_msg.data == 'INDEPENDENT':
                self.drive_mode_msg.data = 'STANDARD'

        # Get the wheel selection (front or back) (relevant for independent mode only)
        if data['buttons'][inputs.UP] == 1:
            self.wheg_msg.data = 'FRONT'
        elif data['buttons'][inputs.DOWN] == 1:
            self.wheg_msg.data = 'BACK'

        # velocity message
        velocity_msg = Twist()

        # must be pressing L2 and R2 to deliver power
        if data['axes'][inputs.RIGHT_TRIGGER] > 0.95 and data['axes'][inputs.LEFT_TRIGGER] > 0.95:

            if (abs(data['axes'][inputs.LEFT_JOY_VERTICAL]) > 0.25):
                velocity_msg.linear.x = data['axes'][inputs.LEFT_JOY_VERTICAL]

            if (abs(data['axes'][inputs.LEFT_JOY_HORIZONTAL]) > 0.25):
                velocity_msg.angular.z = data['axes'][inputs.LEFT_JOY_HORIZONTAL]

            # use linear.y for right joystick (it is still linear.x velocity)
            if (abs(data['axes'][inputs.RIGHT_JOY_VERTICAL]) > 0.25):
                velocity_msg.linear.y = data['axes'][inputs.RIGHT_JOY_VERTICAL]

        self.velocity_publisher_.publish(velocity_msg)
        self.speed_mode_publisher_.publish(self.speed_mode_msg)
        self.drive_mode_publisher_.publish(self.drive_mode_msg)
        self.wheg_selection_publisher_.publish(self.wheg_msg)

    def get_joint_commands(self, data):
        """Process and publish commands for the joints."""
        current_time = time.time()
        debounce_time = 0.5 # seconds

        # toggle between t-joints
        if data['buttons'][inputs.CIRCLE] == 1:

            # Only toggle if the button wasn't previously pressed
            if not self.circle_button_pressed and (current_time - self.circle_last_pressed_time > debounce_time):
                self.circle_last_pressed_time = current_time

                # Toggle the state
                if self.joint_msg.joint.data == 'FRONT':
                    self.joint_msg.joint.data = 'BACK'
                elif self.joint_msg.joint.data == 'BACK':
                    self.joint_msg.joint.data = 'FRONT'

                # Set the flag to indicate the button is now pressed
                self.circle_button_pressed = True

        # Check if the button is released
        elif data['buttons'][inputs.CIRCLE] == 0:
            # Reset the flag when the button is released
            self.circle_button_pressed = False

        # must be pressing L2 and R2 to deliver power
        if data['axes'][inputs.RIGHT_TRIGGER] > 0.95 and data['axes'][inputs.LEFT_TRIGGER] > 0.95:
            
            # only publish up or down at one time
            if self.joint_msg.down != 1:
                self.joint_msg.up = data['buttons'][inputs.R1] # up

            if self.joint_msg.up != 1:
                self.joint_msg.down = data['buttons'][inputs.L1] # down

        else:
            self.joint_msg.up = 0
            self.joint_msg.down = 0
        
        self.joint_publisher_.publish(self.joint_msg)
        
           
def main(args=None):
    rclpy.init(args=args)
    node = ControllerCommandPublisher()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
