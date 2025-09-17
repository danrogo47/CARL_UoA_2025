import rclpy
from rclpy.node import Node
from time import sleep
import logging
import yaml
import time
from datetime import datetime
import os
import atexit
from std_msgs.msg import Bool, String
from custom_msgs.msg import Joint
from carl_controller.wheg_plugin.dynamixel_joint_control import DynamixelJointController

class JointNode(Node):
        
    def __init__(self):
    
        super().__init__('joint') # Remove if not necessary

        with open('config_joint.yaml', 'r') as file:
            self.config = yaml.safe_load(file)
            
        self.setup_logging()
            
        self.SHUT_DOWN = False

        self.dynamixel = DynamixelJointController()
            
        # Initialize pivot angles and limits
        self.setup_pivots()
        
        # Subscribe to controller input topics
        self.subscription_1 = self.create_subscription(Bool, 'shutdown_cmd', self.shutdown_callback, 10)
        self.subscription_2 = self.create_subscription(Joint, 'joint_cmd', self.joint_callback, 10)

    def setup_logging(self):
        # Create Logs directory if it doesn't exist
        log_directory = self.config['logging']['log_directory']
        if not os.path.exists(log_directory):
            os.makedirs(log_directory)

        # Generate log file based on date and time
        log_filename = f"{log_directory}/flik_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log"

        # Set up logging to log motor positions and controller inputs
        logging.basicConfig(
            filename=log_filename,
            level=getattr(logging, self.config['logging']['log_level_file']),  
            format='%(asctime)s %(levelname)s: %(message)s'
        )

        console_handler = logging.StreamHandler()
        console_handler.setLevel(getattr(logging, self.config['logging']['log_level_console']))  # Set console output
        console_formatter = logging.Formatter('%(asctime)s %(levelname)s: %(message)s')
        console_handler.setFormatter(console_formatter)

        # Add the handler to the logger
        logging.getLogger().addHandler(console_handler)

        # Create the CSV file without headers (headers will be added later in write_to_csv)
        self.csv_filename = f"{log_directory}/flik_test_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
        
        # Just create an empty CSV file
        with open(self.csv_filename, mode='w', newline='') as csvfile:
            pass  # CSV file will be populated with headers in write_to_csv

    def setup_pivots(self):
        # Initialize pivot motors and set their parameters
        self.PIVOTS = self.config['motor_ids']['pivots']
        self.front_pivot_angle = self.config['pivot_parameters']['initial_front_angle']
        self.rear_pivot_angle = self.config['pivot_parameters']['initial_rear_angle']
        self.pivot_max_angle = self.config['position_limits']['Hinges']['max_degrees']
        self.pivot_min_angle = self.config['position_limits']['Hinges']['min_degrees']
        self.pivot_step = self.config['pivot_parameters']['pivot_step']
        self.allow_pivot_control = True
    
        # Set position limits for the pivot motors
        self.dynamixel.set_drive_mode_group('Pivot_Group', False)
        self.dynamixel.set_position_limits_group('Pivot_Group', self.config['position_limits']['Hinges']['min_degrees'], self.config['position_limits']['Hinges']['max_degrees'])
        self.dynamixel.set_operating_mode_group('Pivot_Group', 'position')
        logging.info("Set position limits for the pivot motors")

    def shutdown_callback(self, msg):
        # sets the shutdown flag to true if the current sensing chip detects a current spike
        if msg.data:
            self.shutdown_motors()
            
    def motor_shutdown(self):
        """Shutdown motors gracefully."""    
        self.SHUT_DOWN = True    
        self.dynamixel.torque_off_group('Pivot_Group')
    
    def joint_callback(self, msg):
        pivot_change = False
        
        # shutdown flag true: immediately stop motor movement
        if self.SHUT_DOWN:
            self.dynamixel.torque_off_group("Pivot_Group")
            return
        
        # set movement direction based on controller input
        if msg.front_down == 1:
            self.adjust_front_pivot('down')
            pivot_change = True
        elif msg.front_up == 1:
            self.adjust_front_pivot('up')
            pivot_change = True
        elif msg.back_up == 1:
            self.adjust_rear_pivot('up')
            pivot_change = True
        elif msg.back_down == 1:
            self.adjust_rear_pivot('down')
            pivot_change = True
            
        if pivot_change:
            # Prepare positions for sync write
            pivot_positions = {
                self.config['motor_ids']['pivots']['FRONT_PIVOT']: self.front_pivot_angle,
                self.config['motor_ids']['pivots']['REAR_PIVOT']: self.rear_pivot_angle
            }
            
            # Sync write the goal positions for the pivots
            self.dynamixel.set_position_group('Pivot_Group', pivot_positions)
            sleep(0.01)

            # Logging
            logging.info(f"Front pivot angle set to {self.front_pivot_angle} degrees (ticks: {self.front_pivot_angle})")
            logging.info(f"Rear pivot angle set to {self.rear_pivot_angle} degrees (ticks: {self.rear_pivot_angle})")

    def adjust_front_pivot(self, direction):
        """Adjust the front pivot angle based on D-pad input."""
        if direction == 'up':
            self.front_pivot_angle = max(self.front_pivot_angle - self.pivot_step, self.pivot_min_angle)
        elif direction == 'down':
            self.front_pivot_angle = min(self.front_pivot_angle + self.pivot_step, self.pivot_max_angle)
            
    
    def adjust_rear_pivot(self, direction):
        """Adjust the rear pivot angle based on D-pad input."""
        if direction == 'up':
            self.rear_pivot_angle = max(self.rear_pivot_angle - self.pivot_step, self.pivot_min_angle)
        elif direction == 'down':
            self.rear_pivot_angle = min(self.rear_pivot_angle + self.pivot_step, self.pivot_max_angle)


def main(args=None):

    rclpy.init(args=args)
    
    node = JointNode()

    atexit.register(node.motor_shutdown)
    
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
