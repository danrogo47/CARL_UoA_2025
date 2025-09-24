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
import yaml
import carl_controller.lib.controller_input_defs as inputs
import threading # Testing

from custom_msgs.msg import Joint, GaitCommand

class ControllerCommandPublisher(Node):
    """Receives controller commands, converts them to topics and publishes them."""

    def __init__(self):

        super().__init__('controller_command_publisher')

        # ROS topics to publish from the controller inputs        
        self.controller_state_publisher_ = self.create_publisher(String, 'controller_state', 100)
        self.velocity_publisher_ = self.create_publisher(Twist, 'cmd_vel', 100)
        self.speed_mode_publisher_ = self.create_publisher(Float32, 'speed_mode', 10)
        self.joint_publisher_ = self.create_publisher(Joint, 'joint_cmd', 10)
        self.gait_selection_publisher_ = self.create_publisher(GaitCommand, 'gait_selection', 10)
        self.shutdown_publisher_ = self.create_publisher(Int16, 'shutdown_cmd', 10)
        self.resume_publisher_ = self.create_publisher(Int16, 'resume_cmd', 10)

        # set default speed multiplier to 25%
        self.prev_speed_multiplier = 0.25
        
        self.resume_msg = 0
        self.shutdown_msg = 0

        # set debounce time for button presses
        self.debounce_time = 0.5 # seconds
        self.dpad_debounce_time = 0.1 # seconds
        self.circle_last_pressed_time = 0 
        self.square_last_pressed_time = 0
        self.cross_last_pressed_time = 0
        self.triangle_last_pressed_time = 0
        self.ps_last_pressed_time = 0
        self.updown_last_pressed_time = 0
        self.leftright_last_pressed_time = 0
        self.R1_last_pressed_time = 0
        self.L1_last_pressed_time = 0

        # speed mode message
        self.speed_mode_msg = Float32()
        self.speed_mode_msg.data = 1.0
        
        self.joint_msg = Joint()

        # Wheg selection message
        self.wheg_msg = String()
        self.wheg_msg.data = 'FRONT'

        # Gait selection message
        self.gait_selection_msg = GaitCommand()
        self.gait_selection_msg.gait_number = 0  # Default gait selection

        # set flags for buttons pressed
        self.circle_button_pressed = False
        self.cross_button_pressed = False
        self.square_button_pressed = False
        self.triangle_button_pressed = False

        threading.Thread(target=self.receive_data, daemon=True).start() # Testing this over self.receive_data()
        # self.receive_data()

    def receive_data(self):
        # Set the IP address and port for the server
        server_ip = '0.0.0.0'  # Listen on all available network interfaces
        server_port = 8000  # Choose a port number that is not in use

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.bind((server_ip, server_port))
            server_socket.listen(1)  # Listen for incoming connections
            self.get_logger().info(f'Server listening on port {server_port}...')

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
                                    self.get_gait_commands(data_array)
                                    self.get_joint_commands(data_array)


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

        # set time counter
        current_time = time.time()

        # Set the speed multiplier for driving the wheels
        if data['buttons'][inputs.SHARE] == 1:
            self.speed_mode_msg.data = 0.25
        elif data['buttons'][inputs.OPTIONS] == 1:
            self.speed_mode_msg.data = 0.50
        elif data['buttons'][inputs.TOUCH_PAD] == 1:
            self.speed_mode_msg.data = 1.0

        # velocity message (NOT RELEVANT?)
        velocity_msg = Twist()

        # must be pressing L2 and R2 to deliver power
        if data['axes'][inputs.RIGHT_TRIGGER] > 0 and data['axes'][inputs.LEFT_TRIGGER] > 0:
            # Stop conditions
            velocity_msg.linear.x = 0.0
        elif data['axes'][inputs.RIGHT_TRIGGER] > 0:
            # Forward movement
            velocity_msg.linear.x = data['axes'][inputs.RIGHT_TRIGGER]
        elif data['axes'][inputs.LEFT_TRIGGER] > 0:
            # Reverse Movement
            velocity_msg.linear.x = (data['axes'][inputs.LEFT_TRIGGER])*-1
        else:
            # No trigger input, stop the robot
            velocity_msg.linear.x = data['axes'][inputs.RIGHT_TRIGGER]
            
        if data['buttons'][inputs.CIRCLE] == 1 and (current_time - self.circle_last_pressed_time > self.debounce_time):
            self.circle_last_pressed_time = current_time
            self.shutdown_msg = 1
            self.shutdown_publisher_.publish(self.shutdown_msg)
            self.get_logger().info("Shutdown command sent.")
        elif data['buttons'][inputs.CROSS] == 1 and (current_time - self.cross_last_pressed_time > self.debounce_time):
            self.cross_last_pressed_time = current_time
            self.resume_msg = 1
            self.resume_publisher_.publish(self.resume_msg)
            self.get_logger().info("Resume command sent.")

        self.velocity_publisher_.publish(velocity_msg)
        self.speed_mode_publisher_.publish(self.speed_mode_msg)

    def get_gait_commands(self, data):
        """Process and publish commands for the gait."""
        current_time = time.time()
        
        # Move up to the next body compartment when pressing R1
        if(data['buttons'][inputs.R1] == 1) and (current_time - self.R1_last_pressed_time > self.debounce_time):
            self.R1_last_pressed_time = current_time
            
            if self.gait_selection_msg.body_number == 3:
                self.gait_selection_msg.body_number = 1
            else :
                self.gait_selection_msg.body_number += 1
        elif(data['buttons'][inputs.L1] == 1) and (current_time - self.L1_last_pressed_time > self.debounce_time):
            self.L1_last_pressed_time = current_time
            
            if self.gait_selection_msg.body_number == 1:
                self.gait_selection_msg.body_number = 3
            else :
                self.gait_selection_msg.body_number -= 1


        # Decrement the gait selection when pressing square
        if (data['buttons'][inputs.SQUARE] == 1) and (current_time - self.square_last_pressed_time > self.debounce_time):
            self.square_last_pressed_time = current_time

            # toggle the gait mode
            if self.gait_selection_msg.gait_number == 0:
                self.gait_selection_msg.gait_number = 3
            else :
                self.gait_selection_msg.gait_number -= 1

        # Increment the gait selection
        if (data['buttons'][inputs.TRIANGLE] == 1) and (current_time - self.triangle_last_pressed_time > self.debounce_time):
            self.triangle_last_pressed_time = current_time

            # toggle the gait mode
            if self.gait_selection_msg.gait_number == 3:
                self.gait_selection_msg.gait_number = 0
            else :
                self.gait_selection_msg.gait_number += 1
                
        self.gait_selection_publisher_.publish(self.gait_selection_msg)

        

    def get_joint_commands(self, data):
        """Process and publish commands for the joints."""
        current_time = time.time()
        
        if data['buttons'][inputs.UP] == 1 and data['buttons'][inputs.DOWN] == 1:
            self.joint_msg.front_up = 0
            self.joint_msg.front_down = 0
        elif data['buttons'][inputs.LEFT] == 1 and data['buttons'][inputs.RIGHT] == 1:
            self.joint_msg.back_up = 0
            self.joint_msg.back_down = 0
        else:
            # Ensure the pivot angle adjustments can keep up with the motor speed
            if current_time - self.updown_last_pressed_time > self.dpad_debounce_time:
                self.updown_last_pressed_time = current_time
                self.joint_msg.front_up = data['buttons'][inputs.UP]
                self.joint_msg.front_down = data['buttons'][inputs.DOWN]
            if current_time - self.leftright_last_pressed_time > self.dpad_debounce_time:
                self.leftright_last_pressed_time = current_time
                self.joint_msg.back_up = data['buttons'][inputs.RIGHT]
                self.joint_msg.back_down = data['buttons'][inputs.LEFT]
        
        self.joint_publisher_.publish(self.joint_msg)
        
           
def main(args=None):
    rclpy.init(args=args)
    node = ControllerCommandPublisher()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
