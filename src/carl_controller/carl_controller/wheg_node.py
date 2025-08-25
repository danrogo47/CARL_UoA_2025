from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String, Int16
from geometry_msgs.msg import Twist
from custom_msgs.msg import WhegFeedback
from wheg_plugin.gait_controller import GaitController
from dynamixel_control import DynamixelController
import rclpy
import logging
import asyncio
import yaml
import time
from math import pi
import atexit

class WhegMotorDrive(Node):
    def __init__(self):

        super().__init__('wheg_drive')
        
        with open('config.yaml', 'r') as file:
            self.config = yaml.safe_load(file)
            
        try:
            self.dynamixel = DynamixelController()
        except Exception as e:
            logging.error(f"Error initialising dynamixel: {e}")

        # initialise the wheg controller functions
        self.gait = GaitController(self.config)
            
        self.gait.setup_variables()
        self.initialise_direction()
        self.execute_gait_change()

        self.motor_shutdown()

        # shutdown flag
        self.SHUT_DOWN = False

        # update time
        self.dt = 0.1  # seconds

        self.last_called_time = time.time()
        
        self.velocities = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0} # iniitialise velocities for each wheg

        # subscribe to current sensing command
        self.subscription_1 = self.create_subscription(Bool, 'shutdown_cmd', self.shutdown_callback, 10)
        # subscribe to velocity cmds
        self.subscription_2 = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        # subscribe to gait selection
        self.subscription_3 = self.create_subscription(Int16, 'gait_selection', self.gait_mode_callback, 10)
        # subscribe to speed mode
        self.subscription_4 = self.create_subscription(Float32, 'speed_mode', self.speed_mode_callback, 10)
        # subscribe to resume mode
        self.subscription_5 = self.create_subscription(Bool, 'resume_cmd', self.resume_callback, 10)
        # publish motor torques
        self.torque_publisher_ = self.create_publisher(WhegFeedback, 'wheg_feedback', 100)

    def shutdown_callback(self, msg):

        # sets the shutdown flag to true if the current sensing chip detects a current spike
        if msg.data:
            self.SHUT_DOWN = True
            self.gait.set_shutdown(True)
            
    def resume_callback(self):
        
        self.gait.set_shutdown(False)
        
        self.dynamixel.torque_on_group('Wheg_Group')

    def gait_mode_callback(self, msg):
        # TODO : Add a check to ensure the gait index is actually changed

        # sets the speed multiplier for driving
        self.gait.next_gait_index = msg.data
        self.gait_change_requested = True
        self.gait.execute_gait()
        
    def speed_mode_callback(self, msg):

        # sets the speed multipler for driving
        self.speed_multiplier = msg.data
        
    def listener_callback(self, msg):

        if self.SHUT_DOWN:
            self.motor_shutdown()
            return
        
        if self.gait_change_requested:
            self.execute_gait_change()
        
        current_time = time.time()

        # only send commands every set time interval dt
        if (current_time - self.last_called_time) > self.dt:

            self.calculate_gait_velocities(msg)
            self.last_called_time = current_time
            
            self.drive_motors()
            
        # get motor feedback for streamlit
        self.get_torque_feedback()

    def calculate_gait_velocities(self, msg):
        
        if(self.gait_change_requested):
            self.execute_gait_change()
        
        # Speed throttle speed      
        x_cmd = msg.linear.x
        
        self.gait.execute_gait(x_cmd)
        
        # TODO: Calculate the speeds based on the gait
        self.raw_velocities = self.gait.get_velocities() * self.speed_multiplier

        # ease the speeds
        self.velocities = self.ease_speed()
        
    def ease_speed(self, new_speed, prev_speed):
        
        self.raw_velocities # new speed
        self.velocities # prev speed

        for i in self.raw_velocities.keys():
            
            max_delta = 10 * self.dt
            new_speed = self.raw_velocities[i]
            prev_speed = self.velocities[i]
            
            if abs(new_speed - prev_speed) >= max_delta:

                if new_speed > prev_speed:
                    # speeding up
                    corrected_speed = prev_speed + max_delta
                    print(f"corrected_speed: {corrected_speed}")

                else:
                    # slowing down
                    corrected_speed = prev_speed - max_delta
                    print(f"corrected_speed: {corrected_speed}")

            else:
                corrected_speed = new_speed

            self.velocities[i] = corrected_speed
    
    def drive_motors(self):
            # Set profile velocities and increments
            self.dynamixel.set_operating_mode_group('Wheg_Group', 'multi_turn')
            increments = self.gait.get_increments()
            self.dynamixel.set_group_profile_velocity('Wheg_Group', self.velocities)
            self.dynamixel.increment_group_position('Wheg_Group', increments)

    async def get_torque_feedback(self):
        """
        Retrieves motor data including positions, velocities, loads, and error statuses.
        Returns a dictionary with each motor's ID and its associated data.
        Loading can be associated with a motor's torque.
        """
        try:
            # Perform a bulk read for motor positions, velocities, loads, and hardware errors
            motor_positions = self.dynamixel.bulk_read_group('Wheg_Group', ['present_position'])
            motor_velocities = self.dynamixel.bulk_read_group('Wheg_Group', ['present_velocity'])
            motor_loads = self.dynamixel.bulk_read_group('Wheg_Group', ['present_load'])
            hardware_errors = self.dynamixel.bulk_read_group('Wheg_Group', ['hardware_error_status'])

            motor_data = {}

            for motor_id in motor_positions.keys():
                # Retrieve position, velocity, load, and error status
                position_ticks = motor_positions[motor_id].get('present_position', 'N/A')
                velocity = motor_velocities[motor_id].get('present_velocity', 'N/A')
                load = motor_loads[motor_id].get('present_load', 'N/A')
                error_status = hardware_errors[motor_id].get('hardware_error_status', 0)

                # Convert position to degrees, velocity to RPM, and load to percentage
                position_degrees = ((position_ticks * 360) / 4096) % 359 if isinstance(position_ticks, (int, float)) else 'N/A'
                velocity_rpm = (velocity * 0.229) if isinstance(velocity, (int, float)) else 'N/A'
                load_percentage = (load - 65536) / 10.0 if load > 32767 else (load / 10.0 if isinstance(load, (int, float)) else 'N/A')

                # Store processed data in motor_data dictionary
                motor_data[motor_id] = {
                    "position_degrees": position_degrees,
                    "velocity_rpm": velocity_rpm,
                    "load_percentage": load_percentage,
                    "error_status": error_status
                }

            return motor_data

        except Exception as e:
            logging.error(f"Error retrieving motor data: {e}")
            return {}

    def motor_shutdown(self):
        
        self.dynamixel.torque_off_group('Wheg_Group')
        
    def execute_gait_change(self):
        """
        Executes the gait change by calling the gait controller's method.
        This method is called when the gait change is requested.
        """
        self.gait.execute_gait_change()
        
        self.gait.get_positions()
        
        self.dynamixel.set_position_group('Wheg_Group', self.gait.get_positions())
        self.dynamixel.set_operating_mode_group('Wheg_Group', 'multi_turn')
        
        self.gait_change_requested = False
        
    def initialise_direction(self):
        """
        Initialises the direction of the wheg motors based on the configuration.
        This method sets the initial direction for the wheg motors.
        """
        
        try:
            direction = {1 : 0, 2 : 0, 3 : 0, 4 : 1, 5 : 1, 6 : 1}
            self.dynamixel.set_drive_mode_group('Wheg_Group', direction)
        except Exception as e:
            logging.error(f"Failed to set direction: {e}")
            self.dynamixel.set_position_group('Wheg_Group', self.gait.get_shutoff_positions())


def main(args=None):

    rclpy.init(args=args)
    
    node = WhegMotorDrive()

    atexit.register(node.motor_shutdown)
    
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()