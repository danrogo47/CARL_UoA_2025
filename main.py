"""
This script implements the main logic for converting PS4 controller inputs into dynamixel motor commands.
Different gaits are implemented.
The script also logs motor positions and controller inputs to a log file.

Dependencies:
    DynamixelController class from dynamixel_control.py
    PS4Controller class from controller.py
    DYNAMIXELSDK
    init file uploaded to the Open RB-150 Board
"""
# External Imports
import os
import time
import logging
import yaml
import asyncio
import csv
import time
import streamlit as st
from PIL import Image, ImageDraw

# Internal Imports
from datetime import datetime
from controller import PS4Controller
from dynamixel_control import DynamixelController

class FLIKRobot:
    def __init__(self):
        # Load configuration from YAML file
        with open('config.yaml', 'r') as file:
            self.config = yaml.safe_load(file)

        # Setup logging
        self.setup_logging()

        # Create variables from the configuration
        self.setup_variables()

        # Initisalise components
        try:
            self.ps4_controller = PS4Controller()
            self.dynamixel = DynamixelController()
            logging.info("Initialised PS4 controller, Dynamixel, and Robot State")
        except Exception as e:
            logging.error(f"Error initialising components: {e}")

        # Setup the whegs and pivots
        self.setup_whegs()
        self.setup_pivots()
        
        # Initialise the dashboard
        self.init_dashboard()

    def init_dashboard(self):
        # Streamlit setup
        st.title("FLIK Robot Dashboard")

        # Create two columns: Left for indicators, Right for motor loads
        col1, col2 = st.columns(2)

        with col1:
            # Add placeholders for throttle, D-pad, and buttons pressed
            st.subheader("Throttle")
            self.throttle_placeholder = st.empty()

            st.subheader("Current Gait")
            self.current_gait_placeholder = st.empty()

            st.subheader("Emergency Stop")
            self.emergency_stop_placeholder = st.empty()

            st.subheader("Robot Direction")
            self.robot_direction_placeholder = st.empty()

            st.subheader("D-Pad Control")
            self.dpad_placeholder = st.empty()

            st.subheader("Buttons Pressed")
            self.buttons_pressed_placeholder = st.empty()

        with col2:
            st.header("Motor Loads")
            # Set up the motor load placeholders in the right column
            self.motor_placeholders = {name: col2.empty() for name in self.config['motor_groups']['All_Motors']}

        # Initialize dictionaries to track button and D-pad press times
        self.button_press_times = {}
        self.dpad_press_times = {}

    async def update_dashboard(self):
        """ Asynchronously updates the Streamlit dashboard with motor loads and PS4 controller state """
        while True:
            try:
                # Retrieve motor data
                motor_data = await self.get_motor_data()

                # Update motor load bars with custom color-coding using HTML and CSS
                for named_id in self.config['motor_groups']['All_Motors']:
                    # Map named motor ID to numeric ID
                    numeric_id = self.config['motor_ids']['whegs'].get(named_id) or self.config['motor_ids']['pivots'].get(named_id)
                    
                    if numeric_id and numeric_id in motor_data:
                        # Get the absolute load percentage and determine color
                        load_percentage = abs(motor_data[numeric_id]["load_percentage"])
                        
                        if load_percentage > 80:
                            color = "#FF4C4C"  # Red
                        elif load_percentage > 50:
                            color = "#FFA500"  # Orange
                        else:
                            color = "#4CAF50"  # Green

                        # Update the specific motor's progress bar using its placeholder
                        self.motor_placeholders[named_id].markdown(f"""
                            <div style="margin-bottom: 10px;">
                                <strong>Motor {named_id} Load: {load_percentage:.1f}%</strong>
                                <div style="background-color: #e0e0e0; border-radius: 5px; height: 20px; width: 100%;">
                                    <div style="width: {load_percentage}%; background-color: {color}; height: 100%; border-radius: 5px;"></div>
                                </div>
                            </div>
                        """, unsafe_allow_html=True)

                # Update status indicators
                # Current Gait
                current_gait_name = self.gait_methods[self.current_gait_index].__name__.replace('_', ' ').capitalize()
                self.current_gait_placeholder.text(current_gait_name)

                # Emergency Stop Status
                if self.emergency_stop_activated:
                    self.emergency_stop_placeholder.markdown('<span style="color:red; font-weight:bold">Activated</span>', unsafe_allow_html=True)
                else:
                    self.emergency_stop_placeholder.markdown('<span style="color:green; font-weight:bold">Deactivated</span>', unsafe_allow_html=True)

                # Robot Direction
                robot_direction = "Forward" if self.current_direction else "Reverse"
                self.robot_direction_placeholder.text(robot_direction)

                # Check for controller inputs
                self.button_states = self.ps4_controller.get_button_input()
                self.dpad_inputs = self.ps4_controller.get_dpad_input()
                self.l2_trigger, self.r2_trigger = self.ps4_controller.get_trigger_input()
                self.joystick_inputs = self.ps4_controller.get_joystick_input()

                current_time = time.time()

                # Handle emergency stop and gait changes within the dashboard update
                # Check for emergency stop (Circle button)
                if 'circle' in self.button_states and self.button_states['circle']:
                    self.emergency_stop_activated = True
                    await self.async_emergency_stop()

                # Optionally, resume control after emergency stop (X button)
                if 'x' in self.button_states and self.button_states['x'] and self.emergency_stop_activated:
                    self.emergency_stop_activated = False
                    logging.info("Emergency Stop Deactivated. Resuming control...")
                    self.gait_change_requested = True
                    self.next_gait_index = 0

                # Monitor Triangle (increase gait) and Square (decrease gait) buttons for gait change
                if self.button_states['triangle']:
                    self.next_gait_index = (self.current_gait_index + 1) % len(self.gait_methods)
                    self.gait_change_requested = True  # Request a gait change
                    logging.info(f"Triangle pressed. Preparing to change to next gait: {self.next_gait_index+1}")

                if self.button_states['square']:
                    self.next_gait_index = (self.current_gait_index - 1) % len(self.gait_methods)
                    self.gait_change_requested = True  # Request a gait change
                    logging.info(f"Square pressed. Preparing to change to previous gait: {self.next_gait_index+1}")

                if self.button_states['share']:
                    self.direction_change_requested = True  # Request a direction change
                    logging.info(f"Share pressed. Reversing the direction of the whegs")

                if self.button_states['r3']:
                    self.turn_mode_requested = True
                    logging.info(f"R3 pressed. Turning mode requested")
                
                if self.button_states['options']:
                    self.turn_mode_deactivate = True
                    logging.info(f"share pressed. Turning mode deactivating")

                # Update the throttle indicator
                r2_trigger = self.r2_trigger
                throttle_value = ((r2_trigger + 1) / 2) * 100  # Map -1 to 0, 1 to 100
                self.throttle_placeholder.progress(throttle_value / 100.0)

                # Update D-Pad Control display with persistence
                # For each D-pad direction, if pressed, record the current time
                for direction in ['dpad_up', 'dpad_down', 'dpad_left', 'dpad_right']:
                    if self.dpad_inputs.get(direction, False):
                        self.dpad_press_times[direction] = current_time

                # Remove entries older than 1 second
                self.dpad_press_times = {k: v for k, v in self.dpad_press_times.items() if current_time - v < 1.0}

                # Map D-pad directions to actions, correcting the rear pivot up/down
                dpad_actions = []
                if 'dpad_up' in self.dpad_press_times:
                    dpad_actions.append('Front Pivot Up')
                if 'dpad_down' in self.dpad_press_times:
                    dpad_actions.append('Front Pivot Down')
                if 'dpad_left' in self.dpad_press_times:
                    dpad_actions.append('Rear Pivot Up')    # Corrected assignment
                if 'dpad_right' in self.dpad_press_times:
                    dpad_actions.append('Rear Pivot Down')  # Corrected assignment

                if dpad_actions:
                    self.dpad_placeholder.text(f"D-Pad Actions: {', '.join(dpad_actions)}")
                else:
                    self.dpad_placeholder.text("D-Pad Actions: None")

                # Update list of buttons pressed with persistence
                # For each button, if pressed, record the current time
                for button, pressed in self.button_states.items():
                    if pressed:
                        self.button_press_times[button] = current_time

                # Remove entries older than 1 second
                self.button_press_times = {k: v for k, v in self.button_press_times.items() if current_time - v < 1.0}

                # Prepare the list of buttons to display
                buttons_pressed = [button.capitalize() for button in self.button_press_times.keys()]

                self.buttons_pressed_placeholder.text(f"Buttons pressed: {', '.join(buttons_pressed) if buttons_pressed else 'None'}")

                await asyncio.sleep(0.05)

            except Exception as e:
                logging.error(f"Error updating dashboard: {e}")
                await asyncio.sleep(1)
        
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

    def setup_whegs(self):
        # Set the right side whegs to reverse
        self.dynamixel.set_drive_mode_group('Right_Whegs', True)
        self.dynamixel.set_drive_mode_group('Left_Whegs', False)
        logging.info("Set the right side whegs to reverse direction")
    
    def reverse_direction(self):
        # Read the current drive mode for all whegs
        direction = self.dynamixel.bulk_read_group('Wheg_Group', ['drive_mode'])

        # Ensure the correct extraction of drive mode values
        try:
            # Reverse the direction for each motor (0 -> 1, 1 -> 0)
            reversed_direction = {
                motor_id: 0 if drive_data['drive_mode'] == 1 else 1
                for motor_id, drive_data in direction.items()
            }

            # Set the reversed drive mode for each motor
            self.dynamixel.set_drive_mode_group('Wheg_Group', reversed_direction)

            wait_time = 3
            logging.info(f"Reversed the direction of all whegs, waiting for {wait_time} seconds")
            time.sleep(wait_time)

        except Exception as e:
            logging.error(f"Failed to reverse direction: {e}")

        # Change the current direction
        self.current_direction = not self.current_direction
        self.direction_change_requested = False
        self.gait_change_requested = True

    def setup_pivots(self):
        # Set position limits for the pivot motors
        self.dynamixel.set_drive_mode_group('Pivot_Group', False)
        self.dynamixel.set_position_limits_group('Pivot_Group', self.config['position_limits']['Hinges']['min_degrees'], self.config['position_limits']['Hinges']['max_degrees'])
        logging.info("Set position limits for the pivot motors")

    def setup_variables(self):
        # Motor and pivot configurations from the YAML file
        self.WHEGS = self.config['motor_ids']['whegs']
        self.PIVOTS = self.config['motor_ids']['pivots']
        self.MAX_RPM = self.config['wheg_parameters']['max_rpm']
        self.MIN_RPM = self.config['wheg_parameters']['min_rpm']
        self.SMOOTHNESS = self.config['wheg_parameters']['smoothness']
        self.front_pivot_angle = self.config['pivot_parameters']['initial_front_angle']
        self.rear_pivot_angle = self.config['pivot_parameters']['initial_rear_angle']
        self.pivot_max_angle = self.config['position_limits']['Hinges']['max_degrees']
        self.pivot_min_angle = self.config['position_limits']['Hinges']['min_degrees']
        self.pivot_step = self.config['pivot_parameters']['pivot_step']
        self.wheg_rpm = self.config['wheg_parameters']['min_rpm']
        self.current_gait_index = 0
        self.next_gait_index = 0
        self.total_gaits = len(self.config['gaits'])
        self.emergency_stop_activated = False
        self.reboot_requested = False
        self.report_timer = time.time()
        self.gait_change_requested = True
        self.direction_change_requested = False
        self.current_direction = True
        self.allow_pivot_control = True
        # Gait parameters
        self.odd_even = 0
        self.gait_parameters = {}
        self.gait2_params = self.config['gaits'].get('gait_2', {})
        self.gait3_params = self.config['gaits'].get('gait_3', {})
        self.gait4_params = self.config['gaits'].get('gait_4', {})
        self.gait_init_methods = {
            0: self.gait_init_1,
            1: self.gait_init_2,
            2: self.gait_init_3,
            3: self.gait_init_4,
        }

        self.gait_methods = {
            0: self.gait_1,
            1: self.gait_2,
            2: self.gait_3,
            3: self.gait_4,
        }
        # Buttons
        self.button_states = {}
        self.dpad_inputs = {}
        self.l2_trigger = -1.0
        self.r2_trigger = -1.0
        self.joystick_inputs = 0
        self.turn_mode_requested = False
        self.turn_mode_active = False
        self.turn_mode_deactivate = False
        self.current_side = "Right"
            
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

    def adjust_wheg_rpm(self, trigger_value):
        """ Function to adjust the speed of the whegs based on how far the right trigger is pressed. Smooth transition to target RPM. """
        logging.debug(f"Adjusting wheg speed: trigger_value={trigger_value}, current_rpm={self.wheg_rpm}")
        target_rpm = ((trigger_value + 1) / 2) * (self.MAX_RPM - self.MIN_RPM) + self.MIN_RPM # Trigger value ranges from -1 to 1, map this to RPM range
        # Implement smooth transition to target RPM
        if target_rpm > self.wheg_rpm:
            self.wheg_rpm = min(self.wheg_rpm + self.SMOOTHNESS, target_rpm)
            self.wheg_rpm = max(self.wheg_rpm, self.MIN_RPM)
            self.wehg_rpm = min(self.wheg_rpm, self.MAX_RPM)
        else:
            self.wheg_rpm = max(self.wheg_rpm - self.SMOOTHNESS, target_rpm)
        if trigger_value == 0.0: # Low trigger value, ensure velocity is 0
            self.wheg_rpm = 0
        logging.debug(f"Adjusted wheg speed: target_rpm={target_rpm}, current_rpm={self.wheg_rpm}")
        return self.wheg_rpm
    
    def log(self, motor_positions, l2_trigger, r2_trigger, button_states, dpad_input):
        """Log the current robot state, including pivots, whegs, and controller inputs."""
        logging.info(f"Front pivot angle: {self.front_pivot_angle}")
        logging.info(f"Rear pivot angle: {self.rear_pivot_angle}")
        logging.info(f"Wheg RPMs: {self.wheg_rpms}")
        logging.info(f"Motor Positions: {motor_positions}")
        logging.info(f"L2 Trigger: {l2_trigger}, R2 Trigger: {r2_trigger}")
        logging.info(f"Button States: {button_states}")
        logging.info(f"D-Pad Input: {dpad_input}")

    async def control_pivots_with_dpad(self):
        """
        Control the front and rear pivots using the D-pad inputs from the controller.
        
        :param dpad_inputs: A dictionary with the state of each button, including the D-pad.streamlit
        :param config: The YAML configuration containing pivot parameters (pivot_step, min/max angles).
        """
        while self.allow_pivot_control:
            try:
                change = False
                # Adjust front and rear pivots based on D-pad input
                if self.dpad_inputs['dpad_down']:
                    self.adjust_front_pivot('down')
                    change = True
                elif self.dpad_inputs['dpad_up']:
                    self.adjust_front_pivot('up')
                    change = True
                elif self.dpad_inputs['dpad_right']:
                    self.adjust_rear_pivot('up')
                    change = True
                elif self.dpad_inputs['dpad_left']:
                    self.adjust_rear_pivot('down')
                    change = True

                if change:
                    # Prepare positions for sync write
                    pivot_positions = {
                        self.config['motor_ids']['pivots']['FRONT_PIVOT']: self.front_pivot_angle,
                        self.config['motor_ids']['pivots']['REAR_PIVOT']: self.rear_pivot_angle
                    }
                    
                    # Sync write the goal positions for the pivots
                    self.dynamixel.set_position_group('Pivot_Group', pivot_positions)

                    # Logging
                    logging.info(f"Front pivot angle set to {self.front_pivot_angle} degrees (ticks: {self.front_pivot_angle})")
                    logging.info(f"Rear pivot angle set to {self.rear_pivot_angle} degrees (ticks: {self.rear_pivot_angle})")
                    
            except Exception as e:
                logging.error(f"Error controlling pivots: {e}")
            
            await asyncio.sleep(0.25) #Control responsiveness
        
    # Define the initialization for each gait (for whegs only, pivots are disabled)
    async def gait_init_1(self):
        logging.info("Initialising Gait 1")
        self.gait_change_requested = False  # Reset the request flag
        
        # Ensure the correct extraction of drive mode values
        try:
            # Set direction of drives modes to match direction
            if self.current_direction:
                direction = {1 : 0, 2 : 0, 3 : 0, 4 : 1, 5 : 1, 6 : 1}
                self.dynamixel.set_drive_mode_group('Wheg_Group', direction)
                logging.warning("Set direction of whegs to forward")
            else:
                direction = {1 : 1, 2 : 1, 3 : 1, 4 : 0, 5 : 0, 6 : 0}
                self.dynamixel.set_drive_mode_group('Wheg_Group', direction)
                logging.warning("Set direction of whegs to reverse")

            logging.warning("Direction set for all whegs")
        except Exception as e:
            logging.error(f"Failed to set direction: {e}")
            self.positions = { 1: self.gait4_params['high_pos'], 2: self.gait4_params['high_pos'], 3: self.gait4_params['high_pos'], 
                              4: self.gait4_params['high_pos'], 5: self.gait4_params['high_pos'], 6: self.gait4_params['high_pos'] }
            self.dynamixel.set_position_group('Wheg_Group', self.positions)
            self.dynamixel.set_position_group('Pivot_Group', 180)
            wait_time = 1
            logging.info(f"Initialised Gait 1, waiting for {wait_time} seconds")
        
        if self.reboot_requested:
            logging.info("Reboot requested, resetting torque all motors")
            self.reboot_requested = False
            await asyncio.sleep(0.5) # Wait one second
            self.front_pivot_angle = self.config['pivot_parameters']['initial_front_angle'] # Reset the pivot angles
            self.rear_pivot_angle = self.config['pivot_parameters']['initial_rear_angle']
            self.dynamixel.torque_on_group('All_Motors')
        self.wheg_rpm = 0
        self.dynamixel.set_position_group('Wheg_Group', 180)
        self.dynamixel.set_position_group('Pivot_Group', 180)
        wait_time = 1
        logging.info(f"Initialised Gait 1, waiting for {wait_time} seconds")
        await asyncio.sleep(wait_time)
        self.dynamixel.set_operating_mode_group('Wheg_Group', 'multi_turn')
        return

    async def gait_init_2(self):
        logging.info("Initialising Gait 2")
        self.gait_change_requested = False  # Reset the request flag
        # Update the min and max RPM for this gait:
        self.MIN_RPM = self.gait2_params['min_rpm']
        self.MAX_RPM = self.gait2_params['max_rpm']
        self.SMOOTHNESS = self.gait2_params['smoothness']
        self.LOW_POS = self.gait2_params['low_pos']
        self.HIGH_POS = self.gait2_params['high_pos']
        self.TOLERANCE = self.gait2_params['tolerance']
        self.odd_even = 0
        self.wheg_rpm = 0
        self.positions = { 1: self.LOW_POS, 2: self.HIGH_POS, 3: self.LOW_POS, 4: self.HIGH_POS, 5: self.LOW_POS, 6: self.HIGH_POS }
        self.dynamixel.set_position_group('Wheg_Group', self.positions)
        self.dynamixel.set_position_group('Pivot_Group', 180)
        wait_time = 1
        logging.info(f"Initialised Gait 2, waiting for {wait_time} seconds")
        await asyncio.sleep(wait_time)
        self.dynamixel.set_operating_mode_group('Wheg_Group', 'multi_turn')
        return

    async def gait_init_3(self):      
        logging.info("Initialsing Gait 3")
        self.gait_change_requested = False  # Reset the request flag
        
        # Ensure the correct extraction of drive mode values
        try:
            # Set direction of drives modes to match direction
            if self.current_direction:
                direction = {1 : 0, 2 : 0, 3 : 0, 4 : 1, 5 : 1, 6 : 1}
                self.dynamixel.set_drive_mode_group('Wheg_Group', direction)
                logging.warning("Set direction of whegs to forward")
            else:
                direction = {1 : 1, 2 : 1, 3 : 1, 4 : 0, 5 : 0, 6 : 0}
                self.dynamixel.set_drive_mode_group('Wheg_Group', direction)
                logging.warning("Set direction of whegs to reverse")

            logging.warning("Direction set for all whegs")
        except Exception as e:
            logging.error(f"Failed to set direction: {e}")
            self.positions = { 1: self.gait4_params['high_pos'], 2: self.gait4_params['high_pos'], 3: self.gait4_params['high_pos'], 
                              4: self.gait4_params['high_pos'], 5: self.gait4_params['high_pos'], 6: self.gait4_params['high_pos'] }
            self.dynamixel.set_position_group('Wheg_Group', self.positions)
            self.dynamixel.set_position_group('Pivot_Group', 180)
            wait_time = 1
            logging.info(f"Initialised Gait 1, waiting for {wait_time} seconds")
        
        # Update the min, max and smoothness for this gait
        self.MIN_RPM = self.gait3_params['min_rpm']
        self.MAX_RPM = self.gait3_params['max_rpm']
        self.SMOOTHNESS = self.gait3_params['smoothness']
        self.TOLERANCE = self.gait3_params['tolerance']
        self.wheg_rpm = 0
        self.odd_even = 0
        self.positions = { 1: self.gait3_params['high_pos'], 2: self.gait3_params['low_pos'], 3: self.gait3_params['low_pos'], 4: self.gait3_params['high_pos'], 5: self.gait3_params['low_pos'], 6: self.gait3_params['low_pos']}
        self.dynamixel.set_position_group('Wheg_Group', self.positions)
        self.dynamixel.set_position_group('Pivot_Group', 180)
        wait_time = 1
        logging.info(f"Initialised Gait 3, waiting for {wait_time} seconds")
        await asyncio.sleep(wait_time)
        self.dynamixel.set_operating_mode_group('Wheg_Group', 'multi_turn')
        return

    async def gait_init_4(self):
        logging.info("Initialising Gait 4")
        self.gait_change_requested = False  # Reset the request flag
        # Update the min and max RPM for this gait:
        self.MIN_RPM = self.gait4_params['min_rpm']
        self.MAX_RPM = self.gait4_params['max_rpm']
        self.SMOOTHNESS = self.gait4_params['smoothness']
        self.odd_even = 0
        self.wheg_rpm = 0
        # Read the current drive mode for all whegs
        direction = self.dynamixel.bulk_read_group('Left_Whegs', ['drive_mode'])
        
        # Log the structure of the direction data to debug
        logging.info(f"Direction data: {direction}")

        # Ensure the correct extraction of drive mode values
        try:
            # Reverse the direction for each motor (0 -> 1, 1 -> 0)
            reversed_direction = {
                motor_id: 0 if drive_data['drive_mode'] == 1 else 1
                for motor_id, drive_data in direction.items()
            }

            # Set the reversed drive mode for each motor
            self.dynamixel.set_drive_mode_group('Left_Whegs', reversed_direction)
            logging.warning("Reversed the direction of the left whegs")

        except Exception as e:
            logging.error(f"Failed to reverse direction: {e}")
        self.positions = { 1: self.gait4_params['low_pos'], 2: self.gait4_params['high_pos'], 3: self.gait4_params['low_pos'], 4: self.gait4_params['high_pos'], 5: self.gait4_params['low_pos'], 6: self.gait4_params['high_pos'] }
        self.dynamixel.set_position_group('Wheg_Group', self.positions)
        self.dynamixel.set_position_group('Pivot_Group', 180)
        wait_time = 1
        logging.info(f"Initialised Gait 4, waiting for {wait_time} seconds")
        await asyncio.sleep(wait_time)
        self.dynamixel.set_operating_mode_group('Wheg_Group', 'multi_turn')
        return
    
    async def gait_1(self):
        """Execute Gait 1 and return how long to wait before the next step."""
        logging.debug("Executing Gait 1")
        self.wheg_rpm = self.adjust_wheg_rpm(self.r2_trigger)
        if self.wheg_rpm > 1 and self.gait_change_requested == False:
        
            # Set the velocity limit for all whegs
            self.dynamixel.set_group_profile_velocity('Wheg_Group', self.wheg_rpm)
            increment = 360  # Example movement angle
            self.dynamixel.increment_group_position('Wheg_Group', increment)

            # Calculate wait time based on RPM (example formula: degrees moved / (6 * RPM))self
            wait_time = increment / (6 * self.wheg_rpm)
            logging.info(f"Gait 1 step executed at {self.wheg_rpm:.2f}RPM, wait for {wait_time:.2f} seconds")
            return wait_time
        return 0  # No movement, no wait time

    async def gait_2(self):
        """Execute Gait 2 and return how long to wait before the next step."""
        logging.debug("Executing Gait 2")
        self.wheg_rpm = self.adjust_wheg_rpm(self.r2_trigger)
        if self.wheg_rpm > 1 and self.gait_change_requested == False:
            # Example RPM-based alternating gait logic
            if self.odd_even % 2 == 0:
                rpm_1 = self.wheg_rpm
                rpm_2 = self.wheg_rpm * (self.gait2_params['fast_ang'] / self.gait2_params['slow_ang'])
                inc_1 = self.gait2_params['slow_ang']
                inc_2 = self.gait2_params['fast_ang']
            else:
                rpm_1 = self.wheg_rpm * (self.gait2_params['fast_ang'] / self.gait2_params['slow_ang'])
                rpm_2 = self.wheg_rpm
                inc_1 = self.gait2_params['fast_ang']
                inc_2 = self.gait2_params['slow_ang']

            # Get the current motor positions
            current_positions = self.dynamixel.bulk_read_group('Wheg_Group', ['present_position'])

            # Convert the positions from dict to degrees by extracting the 'present_position' key from the dict
            current_positions = {
                motor_id: (pos_data['present_position'] * (360 / 4096))%359
                for motor_id, pos_data in current_positions.items()
            }

            # On even steps
            if self.odd_even % 2 == 0:
                # Check if not all motors are within the tolerance
                if not all(
                    abs(current_positions[motor_id] - self.positions[motor_id]) < self.TOLERANCE 
                    for motor_id in current_positions.keys()
                ):
                    logging.warning(f"Motors are not in the correct positions for Gait 2. Positions: {current_positions}")
                    logging.warning("Waiting for 1 second before checking for movement")
                    await asyncio.sleep(0.1)

                    # Get the current motor positions again after waiting
                    new_positions = self.dynamixel.bulk_read_group('Wheg_Group', ['present_position'])

                    # Convert the positions from dict to degrees
                    new_positions = {
                        motor_id: (pos_data['present_position'] * (360 / 4096))%359
                        for motor_id, pos_data in new_positions.items()
                    }

                    # Check if the motors are still moving
                    if all(abs(new_positions[motor_id] - current_positions[motor_id]) < 1 for motor_id in new_positions.keys()):
                        wait_time = 3 # Wait for 3 seconds to allow for resetting the gait
                        logging.critical("Motors are not moving, reseting positions, and waiting for 3 seconds.")
                        self.dynamixel.set_position_group('Wheg_Group', self.positions)                      
                        await asyncio.sleep(0.5)  # Wait for 0.5 second to allow for resetting the gait
                        self.dynamixel.set_operating_mode_group('Wheg_Group', 'multi_turn')
                    else:
                        logging.info("Motors are moving. Continuing with the gait.")
                        return 0.5  # No wait time, motors are moving correctly

            # Set profile velocities and increments
            velocities = {1: rpm_1, 2: rpm_2, 3: rpm_1, 4: rpm_2, 5: rpm_1, 6: rpm_2}
            increments = {1: inc_1, 2: inc_2, 3: inc_1, 4: inc_2, 5: inc_1, 6: inc_2}
            self.dynamixel.set_group_profile_velocity('Wheg_Group', velocities)
            self.dynamixel.increment_group_position('Wheg_Group', increments)

            # Calculate wait time
            wait_time = (inc_1 / (6 * rpm_1))+self.gait2_params['delay']
            self.odd_even += 1
            logging.info(f"Gait 2 step executed at {self.wheg_rpm:.2f}RPM, wait for {wait_time:.2f} seconds")
            return wait_time
        return 0  # No movement, no wait time

    async def gait_3(self):
        """Execute Gait 3 and return how long to wait before the next step."""
        logging.debug("Executing Gait 3")
        self.wheg_rpm = self.adjust_wheg_rpm(self.r2_trigger)
        
        if self.wheg_rpm > 1 and self.gait_change_requested == False:

            # Example alternating gait logic for three sets of whegs
            if self.odd_even % 3 == 0:
                rpm_1 = self.wheg_rpm
                rpm_2 = self.wheg_rpm
                rpm_3 = self.wheg_rpm
                inc_1 = self.gait3_params['fast_ang']
                inc_2 = self.gait3_params['slow_ang']
                inc_3 = 0
            elif self.odd_even % 3 == 1:
                rpm_1 = self.wheg_rpm
                rpm_2 = self.wheg_rpm
                rpm_3 = self.wheg_rpm
                inc_1 = 0
                inc_2 = self.gait3_params['fast_ang']
                inc_3 = self.gait3_params['slow_ang']
            else:
                rpm_1 = self.wheg_rpm
                rpm_2 = self.wheg_rpm
                rpm_3 = self.wheg_rpm
                inc_1 = self.gait3_params['slow_ang']
                inc_2 = 0
                inc_3 = self.gait3_params['fast_ang']
            
            # Get the current motor positions
            current_positions = self.dynamixel.bulk_read_group('Wheg_Group', ['present_position'])

            # Convert the positions from dict to degrees by extracting the 'present_position' key from the dict
            current_positions = {
                motor_id: (pos_data['present_position'] * (360 / 4096)) % 359
                for motor_id, pos_data in current_positions.items()
            }

            # Check if all motors are within the tolerance on even steps
            if self.odd_even % 3 == 0:
                if not all(
                    abs(current_positions[motor_id] - self.positions[motor_id]) < self.gait3_params['tolerance'] 
                    for motor_id in current_positions.keys()
                ):
                    logging.warning(f"Motors are not in the correct positions for Gait 3. Positions: {current_positions}")
                    logging.warning("Waiting for 1 second before checking for movement")
                    await asyncio.sleep(0.1)

                    # Get the current motor positions again after waiting
                    new_positions = self.dynamixel.bulk_read_group('Wheg_Group', ['present_position'])

                    # Convert the positions from dict to degrees
                    new_positions = {
                        motor_id: (pos_data['present_position'] * (360 / 4096)) % 359
                        for motor_id, pos_data in new_positions.items()
                    }

                    # Check if the motors are still moving
                    if all(abs(new_positions[motor_id] - current_positions[motor_id]) < 1 for motor_id in new_positions.keys()):
                        wait_time = 3  # Wait for 3 seconds to allow for resetting the gait
                        logging.critical("Motors are not moving, resetting positions, and waiting for 3 seconds.")
                        self.dynamixel.set_position_group('Wheg_Group', self.positions)                      
                        await asyncio.sleep(0.5)  # Wait for 0.5 second to allow for resetting the gait
                        self.dynamixel.set_operating_mode_group('Wheg_Group', 'multi_turn')
                    else:
                        logging.info("Motors are moving. Continuing with the gait.")
                        return 0.5  # No wait time, motors are moving correctly

            # Set profile velocities for all whegs
            velocities = {1: rpm_1, 2: rpm_2, 3: rpm_3, 4: rpm_1, 5: rpm_2, 6: rpm_3}
            increments = {1: inc_1, 2: inc_2, 3: inc_3, 4: inc_1, 5: inc_2, 6: inc_3}

            self.dynamixel.set_group_profile_velocity('Wheg_Group', velocities)
            self.dynamixel.increment_group_position('Wheg_Group', increments)

            # Calculate wait time based on the largest movement (300 degrees)
            wait_time = (self.gait3_params['fast_ang'] / (6 * self.wheg_rpm)) + self.gait3_params['delay']
            self.odd_even += 1
            logging.info(f"Gait 3 step executed at {self.wheg_rpm:.2f} RPM, wait for {wait_time:.2f} seconds")
            return wait_time

        return 0  # No movement, no wait time

    async def gait_4(self):
        """Execute Gait 4 and return how long to wait before the next step."""
        logging.debug("Executing Gait 4")
        self.wheg_rpm = self.adjust_wheg_rpm(self.r2_trigger)
        if self.wheg_rpm > 1 and self.gait_change_requested == False:
            # Example RPM-based alternating gait logic
            if self.odd_even % 2 == 0:
                rpm_1 = self.wheg_rpm
                rpm_2 = self.wheg_rpm * (self.gait4_params['fast_ang'] / self.gait4_params['slow_ang'])
                inc_1 = self.gait4_params['slow_ang']
                inc_2 = self.gait4_params['fast_ang']
            else:
                rpm_1 = self.wheg_rpm * (self.gait4_params['fast_ang'] / self.gait4_params['slow_ang'])
                rpm_2 = self.wheg_rpm
                inc_1 = self.gait4_params['fast_ang']
                inc_2 = self.gait4_params['slow_ang']

            # Get the current motor positions
            current_positions = self.dynamixel.bulk_read_group('Wheg_Group', ['present_position'])

            # Convert the positions from dict to degrees by extracting the 'present_position' key from the dict
            current_positions = {
                motor_id: (pos_data['present_position'] * (360 / 4096))%359
                for motor_id, pos_data in current_positions.items()
            }

            # On even steps
            if self.odd_even % 2 == 0:
                # Check if not all motors are within the tolerance
                if not all(
                    abs(current_positions[motor_id] - self.positions[motor_id]) < self.TOLERANCE 
                    for motor_id in current_positions.keys()
                ):
                    logging.warning(f"Motors are not in the correct positions for Gait 4. Positions: {current_positions}")
                    logging.warning("Waiting for 1 second before checking for movement")
                    await asyncio.sleep(0.1)

                    # Get the current motor positions again after waiting
                    new_positions = self.dynamixel.bulk_read_group('Wheg_Group', ['present_position'])

                    # Convert the positions from dict to degrees
                    new_positions = {
                        motor_id: (pos_data['present_position'] * (360 / 4096))%359
                        for motor_id, pos_data in new_positions.items()
                    }

                    # Check if the motors are still moving
                    if all(abs(new_positions[motor_id] - current_positions[motor_id]) < 1 for motor_id in new_positions.keys()):
                        wait_time = 3 # Wait for 3 seconds to allow for resetting the gait
                        logging.critical("Motors are not moving, reseting positions, and waiting for 3 seconds.")
                        self.dynamixel.set_position_group('Wheg_Group', self.positions)                      
                        await asyncio.sleep(3)  # Wait for 0.5 second to allow for resetting the gait
                        self.dynamixel.set_operating_mode_group('Wheg_Group', 'multi_turn')
                    else:
                        logging.info("Motors are moving. Continuing with the gait.")
                        return 0.5  # No wait time, motors are moving correctly

            # Set profile velocities and increments
            velocities = {1: rpm_1, 2: rpm_2, 3: rpm_1, 4: rpm_2, 5: rpm_1, 6: rpm_2}
            increments = {1: inc_1, 2: inc_2, 3: inc_1, 4: inc_2, 5: inc_1, 6: inc_2}
            self.dynamixel.set_group_profile_velocity('Wheg_Group', velocities)
            self.dynamixel.increment_group_position('Wheg_Group', increments)

            # Calculate wait time
            wait_time = (inc_1 / (6 * rpm_1))+self.gait2_params['delay']
            self.odd_even += 1
            logging.info(f"Gait 4 step executed at {self.wheg_rpm:.2f}RPM, wait for {wait_time:.2f} seconds")
            return wait_time
        return 0  # No movement, no wait time
    
    async def init_turn_mode(self):
        logging.info("Initialising Turn Mode")
        self.gait_change_requested = False  # Reset the request flag
        # Update the min and max RPM for this gait:
        self.MIN_RPM = self.gait4_params['min_rpm']
        self.MAX_RPM = self.gait4_params['max_rpm']
        self.SMOOTHNESS = self.gait4_params['smoothness']
        self.odd_even = 0
        self.wheg_rpm = 0
        self.TOLERANCE = self.gait4_params['tolerance']
        self.positions = { 1: self.gait4_params['low_pos'], 2: self.gait4_params['high_pos'], 3: self.gait4_params['low_pos'], 4: self.gait4_params['low_pos'], 5: self.gait4_params['high_pos'], 6: self.gait4_params['low_pos'] }
        self.dynamixel.set_position_group('Wheg_Group', self.positions)
        self.dynamixel.set_position_group('Pivot_Group', 180)
        wait_time = 3
        logging.info(f"Initialised turn mode, waiting for {wait_time} seconds")
        await asyncio.sleep(wait_time)
        self.dynamixel.set_operating_mode_group('Wheg_Group', 'multi_turn')
        self.turn_mode_active = True
        self.turn_mode_requested = False
        return
    
    async def turn_mode(self):
        logging.debug("Execute Turn")
        self.wheg_rpm = self.adjust_wheg_rpm(self.r2_trigger)
        if self.wheg_rpm > 1 and self.gait_change_requested == False:
            # Example RPM-based alternating gait logic
            if self.odd_even % 2 == 0:
                rpm_1 = self.wheg_rpm
                rpm_2 = self.wheg_rpm * (self.gait4_params['fast_ang'] / self.gait4_params['slow_ang'])
                inc_1 = self.gait4_params['slow_ang']
                inc_2 = self.gait4_params['fast_ang']
            else:
                rpm_1 = self.wheg_rpm * (self.gait4_params['fast_ang'] / self.gait4_params['slow_ang'])
                rpm_2 = self.wheg_rpm
                inc_1 = self.gait4_params['fast_ang']
                inc_2 = self.gait4_params['slow_ang']

            # Check the joystick input for turning
            if self.joystick_inputs[0] > 0.1:
                inc_3 = -inc_1
                inc_4 = -inc_2
                if self.current_side != "Right":
                    self.positions = { 1: self.gait4_params['low_pos'], 2: self.gait4_params['high_pos'], 3: self.gait4_params['low_pos'], 4: self.gait4_params['low_pos'], 5: self.gait4_params['high_pos'], 6: self.gait4_params['low_pos'] }
                    self.dynamixel.set_position_group('Wheg_Group', self.positions)
                    self.current_side = "Right"
                time.sleep(0.5)

            elif self.joystick_inputs[0] < 0.1:
                inc_3 = inc_1
                inc_4 = inc_2
                inc_1 = -inc_1
                inc_2 = -inc_2
                if self.current_side != "Left":
                    self.positions = { 1: self.gait4_params['high_pos'], 2: self.gait4_params['low_pos'], 3: self.gait4_params['high_pos'], 4: self.gait4_params['high_pos'], 5: self.gait4_params['low_pos'], 6: self.gait4_params['high_pos'] }
                    self.dynamixel.set_position_group('Wheg_Group', self.positions)
                    self.current_side = "Left"
            else:
                return 0

            # Get the current motor positions
            current_positions = self.dynamixel.bulk_read_group('Wheg_Group', ['present_position'])

            # Convert the positions from dict to degrees by extracting the 'present_position' key from the dict
            current_positions = {
                motor_id: (pos_data['present_position'] * (360 / 4096))%359
                for motor_id, pos_data in current_positions.items()
            }

            # On even steps
            if self.odd_even % 2 == 0:
                # Check if not all motors are within the tolerance
                if not all(
                    abs(current_positions[motor_id] - self.positions[motor_id]) < self.TOLERANCE 
                    for motor_id in current_positions.keys()
                ):
                    logging.warning(f"Motors are not in the correct positions for Gait 4. Positions: {current_positions}")
                    logging.warning("Waiting for 1 second before checking for movement")
                    await asyncio.sleep(0.1)

                    # Get the current motor positions again after waiting
                    new_positions = self.dynamixel.bulk_read_group('Wheg_Group', ['present_position'])

                    # Convert the positions from dict to degrees
                    new_positions = {
                        motor_id: (pos_data['present_position'] * (360 / 4096))%359
                        for motor_id, pos_data in new_positions.items()
                    }

                    # Check if the motors are still moving
                    if all(abs(new_positions[motor_id] - current_positions[motor_id]) < 1 for motor_id in new_positions.keys()):
                        wait_time = 3 # Wait for 3 seconds to allow for resetting the gait
                        logging.critical("Motors are not moving, reseting positions, and waiting for 3 seconds.")
                        self.dynamixel.set_position_group('Wheg_Group', self.positions)                      
                        await asyncio.sleep(3)  # Wait for 0.5 second to allow for resetting the gait
                        self.dynamixel.set_operating_mode_group('Wheg_Group', 'multi_turn')
                    else:
                        logging.info("Motors are moving. Continuing with the gait.")
                        return 0.5  # No wait time, motors are moving correctly

            # Set profile velocities and increments
            velocities = {1: rpm_1, 2: rpm_2, 3: rpm_1, 4: rpm_2, 5: rpm_1, 6: rpm_2}
            increments = {1: inc_1, 2: inc_2, 3: inc_1, 4: inc_4, 5: inc_3, 6: inc_4}
            self.dynamixel.set_group_profile_velocity('Wheg_Group', velocities)
            self.dynamixel.increment_group_position('Wheg_Group', increments)

            # Calculate wait time
            wait_time = (inc_1 / (6 * rpm_1))+self.gait2_params['delay']
            self.odd_even += 1
            logging.info(f"Gait 4 step executed at {self.wheg_rpm:.2f}RPM, wait for {wait_time:.2f} seconds")
            return wait_time
        return 0  # No movement, no wait time

    async def turn_mode_off(self):
        # Get current direction
        if self.current_direction:
            direction = {1 : 0, 2 : 0, 3 : 0, 4 : 0, 5 : 0, 6 : 0}
            self.dynamixel.set_drive_mode_group('All_Whegs', direction)
            logging.warning("Set direction of whegs to forward")
        else:
            direction = {1 : 1, 2 : 1, 3 : 1, 4 : 1, 5 : 1, 6 : 1}
            self.dynamixel.set_drive_mode_group('All_Whegs', direction)
            logging.warning("Set direction of whegs to reverse")
        self.turn_mode_deactivate = False
        self.turn_mode_active = False

    async def async_emergency_stop(self):
        """Asynchronously stop all motors during an emergency."""
        logging.warning("Emergency stop activated! Stopping all motors asynchronously.")
        self.dynamixel.set_velocity_group('All_Motors', 0)  # Stop all motors immediately
        await asyncio.sleep(0.01)  # Let other tasks run (non-blocking)

    async def check_inputs(self):
        """Asynchronously check for controller inputs, including gait change."""
        while True:
            new_button_states = self.ps4_controller.get_button_input()
            self.button_states.clear()
            self.button_states.update(new_button_states)

            new_dpad_inputs = self.ps4_controller.get_dpad_input()
            self.dpad_inputs.clear()
            self.dpad_inputs.update(new_dpad_inputs)

            self.l2_trigger, self.r2_trigger = self.ps4_controller.get_trigger_input()

            # Add logging to monitor inputs
            logging.info(f"check_inputs - Button states: {self.button_states}")
            logging.info(f"check_inputs - D-pad inputs: {self.dpad_inputs}")
            logging.info(f"check_inputs - L2 trigger value: {self.l2_trigger}")

            # Check for emergency stop (Circle button)
            if 'circle' in self.button_states and self.button_states['circle']:
                self.emergency_stop_activated = True
                await self.async_emergency_stop()

            # Optionally, resume control after emergency stop (X button)
            if 'x' in self.button_states and self.button_states['x'] and self.emergency_stop_activated:
                self.emergency_stop_activated = False
                logging.info("Emergency Stop Deactivated. Resuming control...")
                self.gait_change_requested = True
                self.next_gait_index = 0

            # Monitor Triangle (increase gait) and Square (decrease gait) buttons for gait change
            if  self.button_states['triangle']:
                self.next_gait_index = (self.current_gait_index + 1) % len(self.gait_methods)
                self.gait_change_requested = True  # Request a gait change
                logging.info(f"Triangle pressed. Preparing to change to next gait: {self.next_gait_index+1}")

            if  self.button_states['square']:
                self.next_gait_index = (self.current_gait_index - 1) % len(self.gait_methods)
                self.gait_change_requested = True  # Request a gait change
                logging.info(f"Square pressed. Preparing to change to previous gait: {self.next_gait_index+1}")
            
            if  self.button_states['share']:
                self.direction_change_requested = True # Request a direction change
                logging.info(f"Share pressed. Reversing the direction of the whegs")

            # Check for controller disconnection
            if self.button_states is None:
                logging.error("Controller is disconnected. Stopping robot.")
                self.emergency_stop()
                break

            await asyncio.sleep(0.05)  # Non-blocking wait to continue checking inputs

    async def execute_gait(self):
        """Execute the current gait asynchronously, adding a 2-second wait for initialization."""
        while True:
            if not self.emergency_stop_activated:
                # Check for hardware errors before executing the gait
                await self.check_hardware_errors()
                if self.emergency_stop_activated:
                    logging.debug("Emergency stop activated due to hardware error, gait execution paused.")
                    continue  # Skip the rest of the loop if emergency stop is activated
                
                if self.turn_mode_active:
                    logging.debug("Turn mode activated executing turn gait")
                    turn_function = self.turn_mode()
                    wait = await turn_function
                    await asyncio.sleep(wait)
                    if self.turn_mode_deactivate:
                        await self.turn_mode_off()
                        logging.info("Turn mode deactivating")
                    continue # Skip the rest of the loop

                # Get the current gait function and execute it
                gait_function = self.gait_methods[self.current_gait_index]
                wait_time = await gait_function()
                
                # Check if a gait change has been requested
                if self.gait_change_requested:
                    # Initialize the new gait
                    init_gait_function = self.gait_init_methods[self.next_gait_index]
                    await init_gait_function()
                    
                    # Update current gait index
                    self.current_gait_index = self.next_gait_index
                    self.gait_change_requested = False
                    logging.info(f"New gait {self.current_gait_index + 1} is now active.")
                
                # Handle direction change
                if self.direction_change_requested:
                    self.reverse_direction()
                    self.direction_change_requested = False
                    logging.info("Direction of whegs reversed.")
                
                # Handle turn mode
                if self.turn_mode_requested:
                    await self.init_turn_mode()
                    logging.info("Turn mode activated.")

                if wait_time > 0:
                    logging.debug(f"Waiting for {wait_time:.2f} seconds before next gait step")
                    await asyncio.sleep(wait_time)  # Non-blocking wait for the calculated time
            else:
                logging.debug("Emergency stop activated, gait execution paused.")
            
            await asyncio.sleep(0.01)  # Small sleep to allow other tasks to run

    async def check_hardware_errors(self):
        """
        Checks for hardware errors on all motors and handles them appropriately.
        """
        try:
            # Read hardware error statuses
            hardware_errors = self.dynamixel.bulk_read_group('All_Motors', ['hardware_error_status'])
            error_detected = False
            reboot_id = None

            # Ensure hardware_errors contains valid data
            if hardware_errors:
                for motor_id, error_status_dict in hardware_errors.items():
                    error_status = error_status_dict.get('hardware_error_status', 0)
                    if error_status != 0:  # Non-zero error status indicates a hardware error
                        error_detected = True
                        logging.error(f"Hardware error detected on motor {motor_id}: Error code {error_status}")
                        reboot_id = motor_id

                        # Decode the hardware error based on the error status bits
                        if error_status & 0b00000001:  # Bit 0 - Input Voltage Error
                            logging.error(f"Motor {motor_id}: Input Voltage Error detected.")
                        if error_status & 0b00000100:  # Bit 2 - Overheating Error
                            logging.error(f"Motor {motor_id}: Overheating Error detected.")
                        if error_status & 0b00001000:  # Bit 3 - Motor Encoder Error
                            logging.error(f"Motor {motor_id}: Motor Encoder Error detected.")
                        if error_status & 0b00010000:  # Bit 4 - Electrical Shock Error
                            logging.error(f"Motor {motor_id}: Electrical Shock Error detected.")
                        if error_status & 0b00100000:  # Bit 5 - Overload Error
                            logging.error(f"Motor {motor_id}: Overload Error detected.")

            # If any hardware errors are detected, reset motors and change gait
            if error_detected and reboot_id is not None:
                logging.warning(f"Hardware error detected on motor {reboot_id}. Rebooting motor and resetting gait...")
                reboot_success = self.dynamixel.reboot_motor(reboot_id)  # Reboot the motor
                if reboot_success:  # If reboot is successful, request a gait change
                    logging.info(f"Motor {reboot_id} rebooted successfully.")
                    self.reboot_requested = True
                    self.gait_change_requested = True
                    self.next_gait_index = 0  # Set the next gait index to 0
                else:  # Emergency stop if reboot fails
                    logging.warning(f"Warning - Motor {reboot_id} reboot failed. Executing emergency stop.")
                    await self.async_emergency_stop()
        except Exception as e:
            logging.error(f"Error checking hardware errors: {e}")

    async def get_motor_data(self):
        """
        Retrieves motor data including positions, velocities, loads, and error statuses.
        Returns a dictionary with each motor's ID and its associated data.
        """
        try:
            # Perform a bulk read for motor positions, velocities, loads, and hardware errors
            motor_positions = self.dynamixel.bulk_read_group('All_Motors', ['present_position'])
            motor_velocities = self.dynamixel.bulk_read_group('All_Motors', ['present_velocity'])
            motor_loads = self.dynamixel.bulk_read_group('All_Motors', ['present_load'])
            hardware_errors = self.dynamixel.bulk_read_group('All_Motors', ['hardware_error_status'])

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
            
    async def report_states(self, log_interval=5):
        """ Asynchronously logs motor states including positions, velocities, loads, and errors. """
        while True:
            try:
                # Retrieve motor data
                motor_data = await self.get_motor_data()
                
                logging.info(f"{'Motor':<10}{'Position (degrees)':<25}{'Velocity (RPM)':<20}{'Load (%)':<10}")
                for motor_id, data in motor_data.items():
                    logging.info(f"{motor_id:<10}{data['position_degrees']:<25.2f}{data['velocity_rpm']:<20.2f}{data['load_percentage']:<10}")

                    # Log any detected hardware errors
                    error_status = data["error_status"]
                    if error_status != 0:
                        logging.error(f"Hardware error on motor {motor_id}: Error code {error_status}")
                        # Additional error handling logic can go here if needed

                await asyncio.sleep(log_interval)

            except Exception as e:
                logging.error(f"Error while reporting robot states: {e}")
                await asyncio.sleep(1)

    async def write_to_csv(self, log_interval=0.2):
        """
        Asynchronously collect and log critical information from the robot using individual bulk reads per parameter,
        and log the data to the CSV file that was created during setup_logging.

        The headers are added when motor IDs become available after the Dynamixel controller initialization.

        :param log_interval: Time (in seconds) between each report logging.
        """
        # Get the motor IDs for logging
        motor_ids = self.dynamixel.motor_groups.get('All_Motors', [])

        if not motor_ids:
            logging.error("Failed to retrieve motor IDs for CSV logging.")
            return

        # Write the CSV headers now that motor IDs are available
        with open(self.csv_filename, mode='a', newline='') as csvfile:
            fieldnames = ['hh:mm', 'ss:usus']
            for motor_id in motor_ids:
                fieldnames.extend([
                    f'Motor_{motor_id}_Position (degrees)',
                    f'Motor_{motor_id}_Velocity (RPM)',
                    f'Motor_{motor_id}_Load (%)',
                    f'Motor_{motor_id}_Error Status'
                ])
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            # If file is empty, write the headers
            csvfile.seek(0, 2)  # Move to the end of the file
            if csvfile.tell() == 0:
                writer.writeheader()

        # Start the asynchronous logging loop
        while True:
            try:
                # Get the current time in hh:mm format and seconds with microseconds for ss:usus
                current_time = datetime.now()
                time_hhmm = current_time.strftime('%H:%M')
                time_ssusus = current_time.strftime('%S:%f')

                # Create a dictionary to hold the row data
                row_data = {
                    'hh:mm': time_hhmm,
                    'ss:usus': time_ssusus
                }

                # Read motor data individually for each parameter
                motor_positions = self.dynamixel.bulk_read_group('All_Motors', ['present_position'])
                motor_velocities = self.dynamixel.bulk_read_group('All_Motors', ['present_velocity'])
                motor_loads = self.dynamixel.bulk_read_group('All_Motors', ['present_load'])
                hardware_errors = self.dynamixel.bulk_read_group('All_Motors', ['hardware_error_status'])

                # Populate the row data with individual readings for each motor
                for motor_id in motor_ids:
                    # Get position data and convert to degrees
                    position_ticks = motor_positions.get(motor_id, {}).get('present_position', 'N/A')
                    if isinstance(position_ticks, (int, float)):
                        position_degrees = ((position_ticks * 360) / 4096) % 359
                    else:
                        position_degrees = 'N/A'

                    # Get velocity data and convert to RPM
                    velocity_ticks = motor_velocities.get(motor_id, {}).get('present_velocity', 'N/A')
                    if isinstance(velocity_ticks, (int, float)):
                        velocity_rpm = (velocity_ticks * 0.229)
                    else:
                        velocity_rpm = 'N/A'

                    # Convert load to percentage (-1000 ~ 1000 corresponds to -100% ~ 100%)
                    load = motor_loads.get(motor_id, {}).get('present_load', 'N/A')
                    if isinstance(load, (int, float)):
                        # Check if load value is in the 16-bit signed integer range and handle negative values
                        if load > 32767:
                            load = load - 65536
                        # Load is in 0.1% units, so dividing by 10 converts it to a percentage
                        load_percentage = load / 10.0
                    else:
                        load_percentage = 'N/A'

                    # Get hardware error status
                    error_status = hardware_errors.get(motor_id, {}).get('hardware_error_status', 0)

                    # Add motor data to the row
                    row_data[f'Motor_{motor_id}_Position (degrees)'] = position_degrees
                    row_data[f'Motor_{motor_id}_Velocity (RPM)'] = velocity_rpm
                    row_data[f'Motor_{motor_id}_Load (%)'] = load_percentage
                    row_data[f'Motor_{motor_id}_Error Status'] = error_status

                    # Log hardware errors, if any
                    if error_status != 0:
                        logging.error(f"Hardware error detected on motor {motor_id}: Error code {error_status}")
                        if error_status & 0b00000001:
                            logging.error(f"Motor {motor_id}: Input Voltage Error detected.")
                        if error_status & 0b00000100:
                            logging.error(f"Motor {motor_id}: Overheating Error detected.")
                        if error_status & 0b00001000:
                            logging.error(f"Motor {motor_id}: Motor Encoder Error detected.")
                        if error_status & 0b00010000:
                            logging.error(f"Motor {motor_id}: Electrical Shock Error detected.")
                        if error_status & 0b00100000:
                            logging.error(f"Motor {motor_id}: Overload Error detected.")

                # Write the row data to the CSV file
                with open(self.csv_filename, mode='a', newline='') as csvfile:
                    writer = csv.DictWriter(csvfile, fieldnames=row_data.keys())
                    writer.writerow(row_data)

                # Wait for the specified log_interval before the next report
                await asyncio.sleep(log_interval)

            except Exception as e:
                logging.error(f"Error while logging robot states: {e}")
                # In case of an error, wait briefly before retrying
                await asyncio.sleep(1)
                
    async def main_loop(self):
        """Main loop to run the asynchronous tasks, with safe shutdown on KeyboardInterrupt."""
        try:
            await asyncio.gather(
                # self.check_inputs(),    # Run input checking
                self.execute_gait(),    # Run gait execution
                self.control_pivots_with_dpad(),
                # self.write_to_csv(0.2), # Write to the csv every 0.2 seconds
                # self.report_states(5),   # Log states every 5 seconds (customizable interval)"""
                self.update_dashboard() # Update the dashboard
            )
        
        except asyncio.CancelledError:
            # Handle task cancellation gracefully and log that tasks were cancelled
            logging.info("Tasks were cancelled due to shutdown, proceeding to cleanup...")
            # Suppress further propagation of CancelledError
            pass

        except KeyboardInterrupt:
            # This block might not be necessary if KeyboardInterrupt triggers a CancelledError
            logging.info("Program interrupted. Proceeding to shutdown.")

        finally:
            # Safely stop all motors and close connections
            logging.info("Initiating safe shutdown...")
            self.dynamixel.set_velocity_group('All_Motors', {1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0, 8: 0})  # Stop all motors
            self.dynamixel.set_position_group('All_Motors', 180)  # Reset motor positions
            self.ps4_controller.close()  # Close the PS4 controller connection
            self.dynamixel.close()  # Close the Dynamixel controller connection
            logging.info("Shutdown complete.")

if __name__ == "__main__":
    robot = FLIKRobot()
    asyncio.run(robot.main_loop())

