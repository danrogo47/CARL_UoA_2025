from rclpy.node import Node
from std_msgs.msg import Float32, String, Int16
from geometry_msgs.msg import Twist
from time import sleep
from datetime import datetime
from custom_msgs.msg import WhegFeedback, Joint, GaitCommand
from carl_controller.wheg_plugin.gait_controller import GaitController
from carl_controller.wheg_plugin.dynamixel_control import DynamixelController
import rclpy
import logging
from datetime import datetime
import yaml
import time
import os
from math import pi
import atexit

class MotorDrive(Node):
    def __init__(self):

        super().__init__('wheg_drive')
        
        with open('config.yaml', 'r') as file:
            self.config = yaml.safe_load(file)

        self.setup_logging()
        self.debug = 1
        self.log = 0
        
        self.whegs_stopped = False
        self.SHUT_DOWN = False
        
        # initialise the wheg controller functions
        self.gait = GaitController(self.config)
            
        self.gait.setup_variables()
        
        try:
            self.dynamixel = DynamixelController()
            logging.info("Initialised Dynamixel")
        except Exception as e:
            logging.error(f"Failed to initialize DynamixelController: {e}")
            return
        
        self.setup_wheg_motors()
        self.setup_pivots()

        self.initialise_direction()
        self.execute_gait_change()
        self.stand()

        self.motor_shutdown()

        # shutdown flag
        self.SHUT_DOWN = False

        # update time
        self.dt = 0.05  # seconds

        self.last_called_time = time.time()
        
        self.velocities = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0} # iniitialise velocities for each wheg
        self.increment = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0} # iniitialise increment for each wheg
        self.speed_multiplier = 10 # default speed multiplier

        # subscribe to current sensing command
        self.subscription_1 = self.create_subscription(Int16, 'shutdown_cmd', self.shutdown_callback, 10)
        # subscribe to velocity cmds
        self.subscription_2 = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 100)
        # subscribe to gait selection
        self.subscription_3 = self.create_subscription(GaitCommand, 'gait_selection', self.gait_mode_callback, 10)
        # subscribe to speed mode
        self.subscription_4 = self.create_subscription(Float32, 'speed_mode', self.speed_mode_callback, 10)
        # subscribe to resume mode
        self.subscription_5 = self.create_subscription(Int16, 'compliant_morphing_cmd', self.compliant_morphing_callback, 10) # Does this need removing?
        self.subscription_6 = self.create_subscription(Joint, 'joint_cmd', self.joint_callback, 10)
        # publish motor torques
        self.torque_publisher_ = self.create_publisher(WhegFeedback, 'wheg_feedback', 20)
        # Publish feedback at 10 Hz (every 0.1 seconds)
        self.feedback_timer = self.create_timer(0.05, self.publish_feedback_timer)

        
        
    def setup_wheg_motors(self):
        # Set the motors to drive forward
        self.driving_forward = True
        self.spin_mode = False
        self.whegs_stopped = False
        self.dynamixel.set_drive_mode_group('Right_Whegs', True)
        self.is_right_reverse = True
        self.dynamixel.set_drive_mode_group('Left_Whegs', False)
        self.is_left_reverse = False
        self.dynamixel.set_operating_mode_group('Wheg_Group', 'multi_turn')
        if self.log:
            logging.info("Set the right side whegs to reverse direction")
        
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

    def shutdown_callback(self, msg):

        # sets the shutdown flag to true if the current sensing chip detects a current spike
        if msg.data == 1 and not self.SHUT_DOWN:
            self.SHUT_DOWN = True
            self.stand()
            logging.warning("Shutdown command received! Stopping all motors.")
            
        if msg.data == 1 and self.SHUT_DOWN:
            self.SHUT_DOWN = False
            self.dynamixel.torque_on_group('Wheg_Group')
            if self.log:
                logging.info("Resume command received! Resuming motor operation.")
                
    def compliant_morphing_callback(self, msg):
        if msg.data == 1 and not self.compliant_morphing:
            self.compliant_morphing = True
            self.dynamixel.torque_off_group('Pivot_Group')
            if self.log:    
                logging.info("Compliant morphing command received. (Placeholder functionality)")
        elif msg.data == 1 and self.compliant_morphing:
            self.compliant_morphing = False
            self.dynamixel.torque_on_group('Pivot_Group')
            if self.log:
                logging.info("Compliant morphing command not active.")
    
    def publish_feedback_timer(self):
        if not self.SHUT_DOWN:
            # get motor feedback for streamlit
            self.get_torque_feedback()


    def setup_pivots(self):
        # Initialize pivot motors and set their parameters
        self.PIVOTS = self.config['motor_ids']['pivots']
        self.front_pivot_angle = self.config['pivot_parameters']['initial_front_angle']
        self.rear_pivot_angle = self.config['pivot_parameters']['initial_rear_angle']
        self.pivot_max_angle = self.config['position_limits']['Hinges']['max_degrees']
        self.pivot_min_angle = self.config['position_limits']['Hinges']['min_degrees']
        self.pivot_step = self.config['pivot_parameters']['pivot_step']
        self.compliant_morphing = False
    
        # Set position limits for the pivot motors
        self.dynamixel.set_drive_mode_group('Pivot_Group', False)
        self.dynamixel.set_position_limits_group('Pivot_Group', self.config['position_limits']['Hinges']['min_degrees'], self.config['position_limits']['Hinges']['max_degrees'])
        self.dynamixel.set_operating_mode_group('Pivot_Group', 'position')
        if self.log:
            logging.info("Set position limits for the pivot motors")

    def joint_callback(self, msg):
        
        # shutdown flag true: immediately stop motor movement
        if self.SHUT_DOWN:
            self.dynamixel.torque_off_group("Pivot_Group")
            return
        
        if self.compliant_morphing:
            self.dynamixel.torque_off_group("Pivot_Group")
            return
        
        pivot_positions = self.read_present_positions_ticks('Pivot_Group')
        pivot_change = False
        
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
        if self.log:
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

    def gait_mode_callback(self, msg):
        # If gait 2, let it change body compartments
        if msg.gait_number == 2:
            if msg.body_number != self.gait.body_number:
                self.gait.body_number = msg.body_number
                if self.log:
                    logging.info(f"Body compartment changed to: {self.gait.body_number}")
        
        # If gait 3, let it change wheg control
        if msg.gait_number == 3:
            if msg.wheg_number != self.gait.wheg_number:
                self.gait.wheg_number = msg.wheg_number
                if self.log:
                    logging.info(f"Wheg control changed to: Wheg #{self.gait.wheg_number}")
        
        # Check to ensure the gait index is actually changed
        if msg.gait_number == self.gait.current_gait_index:
            return
        
        if msg.gait_number == 4 and not self.spin_mode:
            self.spin_mode = True
            self.safe_change_drive_direction(right_reverse=True, left_reverse=True)
            self.driving_forward = True
            if self.log:
                logging.info("Set to Spin")
        elif self.spin_mode:
            self.spin_mode = False
            self.driving_forward = True
            self.safe_change_drive_direction(right_reverse=True, left_reverse=False)

        # sets the speed multiplier for driving
        self.gait.next_gait_index = msg.gait_number
        self.gait_change_requested = True
        self.execute_gait_change()
        
    def execute_gait_change(self):
        """
        Executes the gait change by calling the gait controller's method.
        This method is called when the gait change is requested.
        """
        self.dynamixel.reboot_all_motors()
        self.gait.execute_gait_change()
        
        self.dynamixel.set_operating_mode_group('Wheg_Group', 'multi_turn')
        self.dynamixel.set_position_group('Wheg_Group', self.gait.get_positions())
        self.gait_change_requested = False
    
        
    def speed_mode_callback(self, msg):
        if self.speed_multiplier == msg.data:
            return
        # sets the speed multipler for driving
        self.speed_multiplier = msg.data
        if self.log:
            logging.info(f"Speed mode changed to: {self.speed_multiplier}")
        
    def listener_callback(self, msg):

        if self.SHUT_DOWN:
            self.motor_shutdown()
            if self.log:
                logging.info("Shutdown active, motors are off.")
            return
        
        if self.gait_change_requested:
            self.stop_whegs()
            return
        
        current_time = time.time()

        # only send commands every set time interval dt
        if (current_time - self.last_called_time) > self.dt:

            velocities, wait_time = self.calculate_gait_velocities(msg)
            
            if wait_time == 0:
                self.stop_whegs()
                return
            
            self.last_called_time = current_time
            
            self.drive_motors(velocities, wait_time)

    def calculate_gait_velocities(self, msg):
        
        # Speed throttle speed 
        x_cmd = msg.linear.x
        z_cmd = msg.angular.z
        # If a change of direction for the whegs is being called, set the first velocity iuncermeent to 0 to allow for gait change to occur before motor spinning
        if not self.spin_mode:
            if x_cmd > 0 and not self.driving_forward:
                # Change to Forward movement
                self.safe_change_drive_direction(right_reverse=True, left_reverse=False)
                self.driving_forward = True
            elif x_cmd < 0 and self.driving_forward:
                # Change to Reverse movement
                self.safe_change_drive_direction(right_reverse=False, left_reverse=True)
                self.driving_forward = False
        elif self.spin_mode:
            if x_cmd > 0 and not self.driving_forward:
                # Change to Forward movement
                self.safe_change_drive_direction(right_reverse=True, left_reverse=True)
                self.driving_forward = True
            elif x_cmd < 0 and self.driving_forward:
                # Change to Reverse movement
                self.safe_change_drive_direction(right_reverse=False, left_reverse=False)
                self.driving_forward = False
        
        x_cmd = abs(x_cmd)
        
        if x_cmd <= 0:
            return {key: 0 for key in self.velocities.keys()}, 0
        
        wait_time = self.gait.execute_gait(x_cmd)
        
        raw_velocities = self.gait.get_velocities()

        if self.log:
            logging.info(f"Received x_cmd: {x_cmd}") 
            logging.info(f"Received raw_velocities: {raw_velocities}")

        # Apply speed multiplier to the raw velocities
        adjusted_velocities = {
            key: val * self.speed_multiplier
            for key, val in raw_velocities.items()
        }
        
        if(z_cmd > 0 or z_cmd < 0):
            adjusted_velocities = self.adjust_for_joystick(adjusted_velocities, z_cmd)
        
        # Ease speed makes the maximum and averagage speed slower...
        # new_velocities = self.ease_speed(adjusted_velocities, prev_speed)
        
        return adjusted_velocities, wait_time
    
    def adjust_for_joystick(self, multiplier_velocities, z_cmd):
        turn_factor = 0.6  # Adjust this factor to control turning sensitivity

        # Calculate proportional reduction based on how far the joystick is turned
        reduction = abs(z_cmd) * turn_factor

        joystick_adjusted_velocities = {}
        for key, val in multiplier_velocities.items():
            adjusted_val = val  # Apply base speed scaling

            # Apply turning effect based on direction
            if z_cmd > 0:  # turning right → slow left side (0–2)
                if key in [4, 5, 6]:
                    adjusted_val *= (1 - reduction)
            elif z_cmd < 0:  # turning left → slow right side (3–5)
                if key in [1, 2, 3]:
                    adjusted_val *= (1 - reduction)

            joystick_adjusted_velocities[key] = adjusted_val

        return joystick_adjusted_velocities

    
    def drive_motors(self, velocities, wait_time):
        
        if self.gait_change_requested:
            return
        
        increments = self.gait.get_increments()

        if self.log:
            logging.info(f"Driving with velocities: {velocities} and increments: {increments}")
        
        self.whegs_stopped = False
        self.dynamixel.set_group_profile_velocity('Wheg_Group', velocities)
        self.dynamixel.increment_group_position('Wheg_Group', increments)
        
    # def ease_speed(self, adjusted_velocities, prev_speed):
    #     if self.log:
    #         logging.info(f"New velocities: {adjusted_velocities}")
    #         logging.info(f"Previous speed: {prev_speed}")

    #     velocities = {}
    #     max_delta = 10 * self.dt

    #     for motor_id, target_speed in adjusted_velocities.items():
    #         previous_speed = prev_speed.get(motor_id, 0.0)
    #         delta = target_speed - previous_speed

    #     if abs(delta) > max_delta:
    #         corrected_speed = previous_speed + max_delta * (1 if delta > 0 else -1)
    #     else:
    #         corrected_speed = target_speed

    #     velocities[motor_id] = corrected_speed

    #     return velocities
    
    def safe_change_drive_direction(self, right_reverse: bool, left_reverse: bool):
        try:
            # 1) Read current absolute ticks
            present_ticks = self.read_present_positions_ticks('Wheg_Group')

            # 2) Torque off all wheg motors
            self.dynamixel.torque_off_group('Wheg_Group')
            time.sleep(0.02)

            # 3) Update drive mode groups
            self.dynamixel.set_drive_mode_group('Right_Whegs', right_reverse)
            self.is_right_reverse = right_reverse
            self.dynamixel.set_drive_mode_group('Left_Whegs', left_reverse)
            self.is_left_reverse = right_reverse

            # 4) Keep motors in multi_turn (or set to multi_turn if you need velocity control to resume)
            self.dynamixel.set_operating_mode_group('Wheg_Group', 'multi_turn')
            time.sleep(0.02)

            # 5) Re-set the raw positions as goal (important so the controller thinks "I'm already here")
            self.dynamixel.set_position_group('Wheg_Group', present_ticks)

            # 6) Re-enable torque
            self.dynamixel.torque_on_group('Wheg_Group')
            time.sleep(0.02)

            if self.log:
                logging.info(f"Drive direction changed safely. Present ticks re-synced: {present_ticks}")

        except Exception as e:
            logging.error(f"safe_change_drive_direction failed: {e}")

    def get_torque_feedback(self):
        try:
            motor_positions = self.dynamixel.bulk_read_group('Wheg_Group', ['present_position'])
            motor_velocities = self.dynamixel.bulk_read_group('Wheg_Group', ['present_velocity'])
            motor_loads = self.dynamixel.bulk_read_group('Wheg_Group', ['present_load'])

            if not all([motor_positions, motor_velocities, motor_loads]):
                logging.error("One or more bulk reads returned None.")
                return {}

            # Collect processed data
            motor_data = {}

            for motor_id in motor_positions.keys():
                position_ticks = motor_positions[motor_id].get('present_position')
                velocity = motor_velocities[motor_id].get('present_velocity')
                load = motor_loads[motor_id].get('present_load')

                position_degrees = ((position_ticks * 360) / 4096) % 359 if isinstance(position_ticks, (int, float)) else 0.0
                velocity_rpm = (velocity * 0.229) if isinstance(velocity, (int, float)) else 0.0
                if isinstance(load, (int, float)):
                    load_percentage = (load - 65536) / 10.0 if load > 32767 else (load / 10.0)
                else:
                    load_percentage = 0.0
                    
                if velocity_rpm > 100 or self.whegs_stopped:
                    velocity_rpm = 0.0

                motor_data[motor_id] = {
                    "position_degrees": position_degrees,
                    "velocity_rpm": velocity_rpm,
                    "load_percentage": load_percentage,
                }

            # --- Convert dict → ROS message ---
            feedback_msg = WhegFeedback()
            feedback_msg.motor_id = list(motor_data.keys())
            feedback_msg.position_degrees = [v["position_degrees"] for v in motor_data.values()]
            feedback_msg.velocity_rpm = [v["velocity_rpm"] for v in motor_data.values()]
            feedback_msg.load_percentage = [v["load_percentage"] for v in motor_data.values()]
            feedback_msg.right_reverse = int(self.is_right_reverse)
            feedback_msg.left_reverse = int(self.is_left_reverse)

            self.torque_publisher_.publish(feedback_msg)
            if self.log:
                logging.info(f"Published torque feedback {feedback_msg.position_degrees} message successfully.")

        except Exception as e:
            logging.error(f"Error retrieving motor data: {e}")
            import traceback
            logging.debug(traceback.format_exc())
            return {}

    def motor_shutdown(self):
        self.dynamixel.torque_off_group('Wheg_Group')
        
    def initialise_direction(self):        
        try:
            direction = {1 : 0, 2 : 0, 3 : 0, 4 : 1, 5 : 1, 6 : 1}
            self.dynamixel.set_drive_mode_group('Wheg_Group', direction)
            self.is_left_reverse = False
            self.is_right_reverse = True
        except Exception as e:
            logging.error(f"Failed to set direction: {e}")
            
    def stop_whegs(self):
        self.whegs_stopped = True
        """Stop all wheg motors immediately."""
        self.dynamixel.set_group_profile_velocity('Wheg_Group', {i: 0 for i in range(1, 7)})
        self.dynamixel.set_position_group('Wheg_Group', self.read_present_positions_ticks('Wheg_Group'))
        if self.log:
            logging.info("All wheg motors stopped.")

    def stand(self):
        """Set all whegs to a standing position."""
        self.current_positions = { 1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0 }
        self.dynamixel.set_position_group('Wheg_Group', self.current_positions)
        self.dynamixel.set_position_group('Pivot_Group', 180)
        sleep(1)  # Allow time for the motors to reach the standing position
        
    def read_present_positions_ticks(self, group_name: str):
        pos_dict = self.dynamixel.bulk_read_group(group_name, ['present_position'])

        if pos_dict is None:
            if self.log:
                logging.error(f"Failed to read positions from {group_name}")
            return {}

        return {mid: pos_data['present_position'] for mid, pos_data in pos_dict.items()}


def main(args=None):

    rclpy.init(args=args)
    
    node = MotorDrive()

    atexit.register(node.motor_shutdown)
    
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
