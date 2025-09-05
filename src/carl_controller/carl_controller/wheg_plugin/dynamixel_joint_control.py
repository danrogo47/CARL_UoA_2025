""" This script uses the dynamixel_sdk library to control Dynamixel motors connected to the Raspberry Pi via an Open RB15 motor controller board. It uses sync write and bulk write commands to control multiple motors simultaneously. """
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import logging
import yaml
class DynamixelJointController:
    def __init__(self, config_path="config_joint.yaml", device_name=None, baudrate=None, protocol_version=2.0):
        """Initialize the controller with YAML config and setup motor groups."""
        # Load configuration
        self.load_config(config_path)
        
        # Dynamically set the device_name and baudrate from config or passed arguments
        self.device_name = device_name or self.config['controller'].get('device_name')
        self.baudrate = baudrate or self.config['controller'].get('baudrate')

        # Check if device_name or baudrate are not set
        if not self.device_name:
            logging.error("Device name not provided and not found in config")
            raise ValueError("Device name must be provided either as an argument or in the config file")
        
        if not self.baudrate:
            logging.error("Baudrate not provided and not found in config")
            raise ValueError("Baudrate must be provided either as an argument or in the config file")

        # Initialize SDK handlers
        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(protocol_version)

        # Open the port and set the baudrate
        self.open_port()
        
        # Set up motor groups and control table from config
        self.motor_groups = {}  # Initialize as an empty dictionary
        self.load_control_table()
        self.setup_motor_groups()

    def load_config(self, config_path):
        """Load configuration from a YAML file."""
        try:
            with open(config_path, 'r') as file:
                self.config = yaml.safe_load(file)
            logging.info("Configuration loaded successfully")
        except Exception as e:
            logging.error(f"Failed to load config: {e}")
            raise
    
    def open_port(self):
        """Open the port and set baudrate."""
        if not self.port_handler.openPort():
            logging.error("Failed to open the port")
            raise Exception("Failed to open the port")
        logging.info(f"Port {self.device_name} opened successfully")
        
        if not self.port_handler.setBaudRate(self.baudrate):
            logging.error("Failed to set baudrate")
            raise Exception("Failed to set baudrate")
        logging.info(f"Baudrate set to {self.baudrate}")
        
    def create_motor_group(self, group_name, motor_ids):
        """Create a group of motors for easier control."""
        self.motor_groups[group_name] = motor_ids
        logging.info(f"Motor group '{group_name}' created with motors: {motor_ids}")

    def setup_motor_groups(self):
        """Setup all motor groups from the YAML config."""
        motor_groups = self.config['motor_groups']  # Access motor groups from config
        motor_ids = self.config['motor_ids']  # Access motor IDs from config

        for group_name, motor_names in motor_groups.items():
            # Get motor IDs by looking up motor names in motor_ids (whegs and pivots)
            motor_ids_list = [motor_ids['pivots'].get(name) for name in motor_names]
            self.create_motor_group(group_name, motor_ids_list)

    def load_control_table(self):
        """Load control table from the configuration file."""
        self.control_table = self.config.get('control_table', {})
        if not self.control_table:
            logging.error("Control table missing or empty in configuration.")
            raise Exception("Control table not defined in config.")
        
        # Verify each control table entry has both 'address' and 'length'
        for entry, value in self.control_table.items():
            if 'address' not in value or 'length' not in value:
                logging.error(f"Control table entry '{entry}' is missing 'address' or 'length'.")
                raise Exception(f"Control table entry '{entry}' is incomplete.")
        
        logging.info("Control table loaded successfully.")

    def get_control_table_address(self, key):
        """Get the address for a control table key."""
        if key in self.control_table:
            return self.control_table[key]
        else:
            logging.error(f"Control table key '{key}' not found")
            raise ValueError(f"Control table key '{key}' not found")
    
    def position_to_degrees(self, position_value):
        """Convert a raw motor position value (0-4095) to degrees (0-360)."""
        if position_value is None:
            logging.error("Invalid position value: None")
            return None
        return (position_value / 4095.0) * 360.0
    
    def degrees_to_position(self, degrees):
        """Convert degrees to a raw motor position value (0-4095)."""
        if degrees is None:
            logging.error("Invalid degrees value: None")
            return None
        return int((degrees / 360.0) * 4095)

    def set_status_return_level(self, group_name, level=1):
        """Set the status return level for a group of motors."""
        if group_name not in self.motor_groups:
            logging.error(f"Motor group {group_name} not found")
            return
        
        # Set the status return level for all motors in the group
        status_levels = {motor_id: level for motor_id in self.motor_groups[group_name]}
        self.sync_write_group(group_name, 'status_return_level', status_levels)

        logging.info(f"Status return level set to {level} for group {group_name}")

    def sync_write_group(self, group_name, parameter_name, param_dict):
        """
        Sync write command for a group of motors with different values.
        
        :param group_name: The name of the motor group (from config) to which the command will be sent.
        :param parameter_name: The control table parameter to write (e.g., 'position_goal', 'velocity_goal').
        :param param_dict: A dictionary mapping motor_id to the target value for each motor.
        """
        motor_ids = self.motor_groups.get(group_name, [])
        if not motor_ids:
            logging.warning(f"No motors found for group '{group_name}'")
            return
        
        # Retrieve the control table address and length for the given parameter
        control_item = self.control_table.get(parameter_name)
        if not control_item:
            logging.error(f"Control table entry '{parameter_name}' not found.")
            raise Exception(f"Control table entry '{parameter_name}' not found.")
        
        address = control_item['address']
        length = control_item['length']
        
        # Create a GroupSyncWrite instance
        sync_write = GroupSyncWrite(self.port_handler, self.packet_handler, address, length)
        
        # Prepare sync write data for each motor
        for motor_id in motor_ids:
            param = param_dict.get(motor_id)

            if param is not None:
                # Convert floats to integers if needed
                if isinstance(param, float):
                    param = int(param)

                # Convert the value to the appropriate byte format (little endian)
                if length == 1:
                    param_data = [DXL_LOBYTE(param)]
                elif length == 2:
                    param_data = [DXL_LOBYTE(param), DXL_HIBYTE(param)]
                elif length == 4:
                    param_data = [
                        DXL_LOBYTE(DXL_LOWORD(param)),
                        DXL_HIBYTE(DXL_LOWORD(param)),
                        DXL_LOBYTE(DXL_HIWORD(param)),
                        DXL_HIBYTE(DXL_HIWORD(param))
                    ]
                else:
                    logging.error(f"Unsupported data length {length} for parameter '{parameter_name}'.")
                    raise Exception(f"Unsupported data length {length}.")

                # Add the parameter to the sync write for the specific motor
                if not sync_write.addParam(motor_id, bytes(param_data)):
                    logging.error(f"Failed to add parameter for motor {motor_id}.")
                    raise Exception(f"Failed to add parameter for motor {motor_id}.")
        
        # Execute the sync write command
        result = sync_write.txPacket()
        if result != COMM_SUCCESS:
            logging.error(f"Sync write failed with error: {self.packet_handler.getTxRxResult(result)}")

        # Clear the parameters after the sync write
        sync_write.clearParam()

    def set_status_return_level_group(self, group_name, level=2):
        """
        Set the status return level for a group of motors.
        :param group_name: The name of the motor group.
        :param level: The status return level (0: no return, 1: return only for read commands, 2: return for all commands).
        """
        if group_name not in self.motor_groups:
            logging.error(f"Motor group {group_name} not found")
            return

        status_return_params = {motor_id: level for motor_id in self.motor_groups[group_name]}
        self.sync_write_group(group_name, 'status_return_level', status_return_params)
        logging.info(f"Status return level set to {level} for group {group_name}")

    def bulk_read_group(self, group_name, parameters):
        """
        Bulk read command for a group of motors based on the motor group name.
        
        :param group_name: The name of the motor group (from config) to which the command will be sent.
        :param parameters: List of parameters to read (e.g., ['present_position', 'present_velocity']).
        :return: Dictionary where keys are motor_ids and values are dictionaries of parameter data.
        """
        # Retrieve the motor IDs for the given group name
        motor_ids = self.motor_groups.get(group_name, [])
        if not motor_ids:
            logging.warning(f"No motors found for group '{group_name}'")
            return None

        bulk_read = GroupBulkRead(self.port_handler, self.packet_handler)

        logging.debug(f"Preparing bulk read for motors: {motor_ids}, Parameters: {parameters}")

        # Add parameters for each motor in the group
        for motor_id in motor_ids:
            for parameter_name in parameters:
                control_item = self.control_table.get(parameter_name)
                if not control_item:
                    logging.error(f"Control table entry '{parameter_name}' not found.")
                    raise Exception(f"Control table entry '{parameter_name}' not found.")

                # Get the control table address and length for the parameter
                address = control_item['address']
                length = control_item['length']

                logging.debug(f"Adding motor {motor_id}, Address: {address}, Length: {length} to bulk read")

                if not bulk_read.addParam(motor_id, address, length):
                    logging.error(f"Failed to add motor {motor_id} for parameter '{parameter_name}'.")
                    raise Exception(f"Failed to add motor {motor_id} for parameter '{parameter_name}'.")

        # Execute the bulk read command
        result = bulk_read.txRxPacket()
        if result != COMM_SUCCESS:
            logging.error(f"Bulk read failed with error: {self.packet_handler.getTxRxResult(result)}")
            return None

        # Retrieve the motor data
        motor_data = {}
        for motor_id in motor_ids:
            motor_data[motor_id] = {}
            for parameter_name in parameters:
                control_item = self.control_table.get(parameter_name)
                length = control_item['length']

                # Retrieve the raw data from the bulk read
                data = bulk_read.getData(motor_id, control_item['address'], length)

                if data is None:
                    logging.error(f"No data received for motor {motor_id}.")
                    motor_data[motor_id][parameter_name] = None
                else:
                    motor_data[motor_id][parameter_name] = data  # Keep raw data as-is

        bulk_read.clearParam()

        logging.debug(f"Bulk read successful for group '{group_name}'")
        return motor_data

    def set_operating_mode_group(self, group_name, mode):
        """
        Set the operating mode for a group of motors using sync write.
        Available modes: 'position', 'velocity', 'multi_turn'
        :param group_name: The name of the motor group to set the operating mode for.
        :param mode: The operating mode to set ('position', 'velocity', 'multi_turn').
        """
        logging.info(f"Setting operating mode '{mode}' for group '{group_name}'")
        OPERATING_MODES = {
            'position': 3,   # Position control mode
            'velocity': 1,   # Velocity control mode
            'multi_turn': 4  # Multi-turn mode for continuous rotation
        }

        # Validate operating mode
        if mode not in OPERATING_MODES:
            logging.error(f"Invalid operating mode: {mode}")
            return

        # Ensure the motor group exists
        if group_name not in self.motor_groups:
            logging.error(f"Motor group {group_name} not found")
            return

        mode_value = OPERATING_MODES[mode]

        # Disable torque for the entire group before changing the mode
        self.torque_off_group(group_name)

        # Use sync_write_group to set the operating mode for each motor in the group
        operating_mode_params = {motor_id: mode_value for motor_id in self.motor_groups[group_name]}
        self.sync_write_group(group_name, 'operating_mode', operating_mode_params)

        logging.debug(f"Operating mode set to '{mode}' for group {group_name}")

        # Re-enable torque for the entire group after setting the mode
        self.torque_on_group(group_name)

        # Apply specific configuration for velocity and position modes
        if mode == 'velocity':
            logging.debug(f"Setting velocity limit for motors in group {group_name}")
            self.set_group_velocity_limit(group_name)
        else:
            logging.debug(f"Setting profile velocity for motors in group {group_name}")
            self.set_group_profile_velocity(group_name)

    def set_group_velocity_limit(self, group_name):
        """
        Set velocity limit for a group of motors based on config_wheg/joint.yaml or the hard velocity limit.
        
        :param group_name: The name of the motor group to set the velocity limit for.
        """
        if group_name not in self.motor_groups:
            logging.error(f"Motor group {group_name} not found")
            return

        # Get hard velocity limit from config_wheg/joint.yaml
        hard_velocity_limit = self.config.get('hard_velocity_limit', None)
        if hard_velocity_limit is None:
            logging.error(f"Hard velocity limit not found in config_wheg/joint.yaml")
            return

        # Get velocity limit from config_wheg/joint.yaml
        velocity_limit = self.config.get('velocity_limits', {}).get(group_name, None)
        if velocity_limit is None:
            logging.error(f"Velocity limit for group {group_name} not found in config_wheg/joint.yaml")
            return

        # Check if the velocity limit exceeds the hard limit
        if velocity_limit > hard_velocity_limit:
            logging.warning(f"Velocity limit {velocity_limit} exceeds hard limit {hard_velocity_limit}. Limiting to {hard_velocity_limit}.")
            velocity_limit = hard_velocity_limit

        # Apply velocity limit using sync write
        velocity_limits = {motor_id: velocity_limit for motor_id in self.motor_groups[group_name]}
        self.sync_write_group(group_name, 'velocity_limit', velocity_limits)

        logging.info(f"Velocity limit set to {velocity_limit} for group {group_name}")

    def set_group_profile_velocity(self, group_name, profile_velocity=None):
        """
        Set profile velocity for a group of motors based on config_wheg/joint.yaml or a provided value.
        
        :param group_name: The name of the motor group to set the profile velocity for.
        :param profile_velocity: Optional, can be an integer for all motors or a dictionary with motor IDs as keys and velocities as values.
        """
        logging.debug(f"Setting profile velocity for group {group_name}")
        if group_name not in self.motor_groups:
            logging.error(f"Motor group {group_name} not found")
            return

        # Get hard profile velocity limit from config_wheg/joint.yaml
        hard_profile_velocity_limit = self.config.get('hard_profile_velocity_limit', None)
        if hard_profile_velocity_limit is None:
            logging.error(f"Hard profile velocity limit not found in config_wheg/joint.yaml")
            return
        logging.debug(f"Hard profile velocity limit: {hard_profile_velocity_limit}")

        # If no profile_velocity provided, get it from config_wheg/joint.yaml
        if profile_velocity is None:
            profile_velocity = self.config.get('profile_velocities', {}).get(group_name, None)
            logging.info(f"Setting the profile velocity for group {group_name} from config_wheg/joint.yaml: {profile_velocity}")
            if profile_velocity is None:
                logging.error(f"Profile velocity for group {group_name} not found in config_wheg/joint.yaml and no value was provided.")
                profile_velocity = 5
                return

        # Handle the case where profile_velocity is a dictionary (per motor setting)
        if isinstance(profile_velocity, dict):
            # Iterate through the dictionary and apply velocity limits
            profile_velocities = {}
            for motor_id, velocity in profile_velocity.items():
                if velocity > hard_profile_velocity_limit:
                    logging.warning(f"Profile velocity {velocity} for motor {motor_id} exceeds hard limit {hard_profile_velocity_limit}. Limiting to {hard_profile_velocity_limit}.")
                    velocity = hard_profile_velocity_limit
                profile_velocities[motor_id] = velocity
        else:
            # If profile_velocity is a single integer, apply the same velocity to all motors in the group
            if profile_velocity > hard_profile_velocity_limit:
                logging.warning(f"Profile velocity {profile_velocity} exceeds hard limit {hard_profile_velocity_limit}. Limiting to {hard_profile_velocity_limit}.")
                profile_velocity = hard_profile_velocity_limit
            # Apply the same profile velocity to all motors in the group
            profile_velocities = {motor_id: profile_velocity for motor_id in self.motor_groups[group_name]}

        # Convert profile velocities to raw values and ensure they are >= 1
        profile_velocities = {motor_id: max(3, velocity * 4.37) for motor_id, velocity in profile_velocities.items()}

        # Apply profile velocities using sync write
        self.sync_write_group(group_name, 'profile_velocity', profile_velocities)

        logging.info(f"Profile velocities set for group {group_name}: {profile_velocities}")

    def torque_off_group(self, group_name):
        """Disable torque for all motors in the group."""
        logging.info(f"Disabling torque for group {group_name}")
        if group_name not in self.motor_groups:
            logging.error(f"Motor group {group_name} not found")
            return

        # Disable torque for each motor
        torque_values = {motor_id: 0 for motor_id in self.motor_groups[group_name]}
        self.sync_write_group(group_name, 'torque_enable', torque_values)
        logging.debug(f"Torque disabled for group {group_name}")

    def torque_on_group(self, group_name):
        """Enable torque for all motors in the group."""
        logging.info(f"Enabling torque for group {group_name}")
        if group_name not in self.motor_groups:
            logging.error(f"Motor group {group_name} not found")
            return

        # Enable torque for each motor
        torque_values = {motor_id: 1 for motor_id in self.motor_groups[group_name]}
        self.sync_write_group(group_name, 'torque_enable', torque_values)
        logging.debug(f"Torque enabled for group {group_name}")

        # Set a default profile velocity for the group after enabling torque
        self.set_group_profile_velocity(group_name)

    def set_position_group(self, group_name, positions):
        """
        Set target positions (in degrees) for a group of motors.
        
        :param group_name: The motor group name (from config)
        :param positions: Either a dictionary with motor_id as key and target position in degrees as value,
                        or a single integer to apply the same position to all motors in the group.
        """
        logging.info(f"Running set_position_group for group '{group_name}' with positions: {positions}")
        if group_name not in self.motor_groups:
            logging.error(f"Motor group {group_name} not found")
            return

        # Check the operating modes for all motors in the group
        operating_modes = self.bulk_read_group(group_name, ['operating_mode'])
        # If any motor is not in position control mode, set the mode now
        for motor_id, data in operating_modes.items():
            current_mode = data.get('operating_mode', None)
            if current_mode != 3:
                logging.warning(f"Motor {motor_id} is not in Position Control Mode. Setting mode now.")
                self.set_operating_mode_group(group_name, 'position')
                break
                   
        # Check if the input is a dictionary (positions for each motor) or a single integer (same position for all motors)
        if isinstance(positions, int):
            # Apply the same position to all motors
            position_goals = {motor_id: self.degrees_to_position(positions) for motor_id in self.motor_groups[group_name]}
            logging.debug(f"Setting position {positions}° for all motors in group '{group_name}'")
        elif isinstance(positions, dict):
            # Apply different positions for each motor
            position_goals = {}
            for motor_id, degrees in positions.items():
                # Log each conversion for detailed debugging
                raw_position = self.degrees_to_position(degrees)
                logging.debug(f"Setting motor {motor_id} to {degrees}° ({raw_position} ticks)")
                position_goals[motor_id] = raw_position
            logging.debug(f"Setting individual positions for motors in group '{group_name}': {positions}")
        else:
            logging.error("Invalid type for 'positions'. Must be either an integer or a dictionary.")
            return

        # Now let's sync write the positions
        try:
            self.sync_write_group(group_name, 'goal_position', position_goals)
            logging.debug(f"Target positions set for group '{group_name}': {positions}")
        except Exception as e:
            logging.error(f"Failed to set positions for group '{group_name}': {e}")

    def set_velocity_group(self, group_name, velocities):
        """
        Set target velocities for a group of motors.

        :param group_name: The motor group name (from config)
        :param velocities: Dictionary with motor_id as key and target velocity as value, or a single integer to set the same velocity for all motors.
        """
        if group_name not in self.motor_groups:
            logging.error(f"Motor group {group_name} not found")
            return

        # Ensure the group is in velocity control mode
        self.set_operating_mode_group(group_name, 'velocity')

        # Ensure torque is enabled after mode change
        self.torque_on_group(group_name)

        # Check if any velocity exceeds the hard velocity limit
        hard_velocity_limit = self.config.get('hard_velocity_limit', None)
        if hard_velocity_limit is None:
            logging.error(f"Hard velocity limit not found in config.yaml")
            return

        # If velocities is an integer, set the same velocity for all motors in the group
        if isinstance(velocities, int):
            velocities_dict = {motor_id: velocities for motor_id in self.motor_groups[group_name]}
        elif isinstance(velocities, dict):
            velocities_dict = velocities
        else:
            logging.error(f"Invalid velocities input: must be an int or dict, got {type(velocities)}")
            return

        # Check if any velocity exceeds the hard velocity limit and cap them if necessary
        for motor_id, velocity in velocities_dict.items():
            if velocity > hard_velocity_limit:
                logging.warning(f"Velocity {velocity} for motor {motor_id} exceeds hard limit {hard_velocity_limit}. Limiting to {hard_velocity_limit}.")
                velocities_dict[motor_id] = hard_velocity_limit

        try:
            # Write the velocity values to the motors
            self.sync_write_group(group_name, 'goal_velocity', velocities_dict)
            logging.info(f"Target velocities set for group {group_name}: {velocities_dict}")
        except Exception as e:
            logging.error(f"Failed to set velocities for group {group_name}: {e}")

    def set_drive_mode_group(self, group_name, reverse_direction=False):
        """
        Set the drive mode for a group of motors. The direction can either be the same for all motors
        (when reverse_direction is an int or bool) or specified per motor (when reverse_direction is a dict).

        :param group_name: The motor group name (from motor_groups configuration)
        :param reverse_direction: Either a bool/int (same direction for all motors) or a dict (per motor control).
        """
        motor_ids = self.motor_groups.get(group_name, [])
        if not motor_ids:
            logging.warning(f"No motors found for group '{group_name}'")
            return

        # Initialize param_dict to store motor_id and drive_mode_value pairs
        param_dict = {}

        # Handle case where reverse_direction is a dict (per motor direction control)
        if isinstance(reverse_direction, dict):
            for motor_id in motor_ids:
                # Get the reverse direction for each motor, defaulting to False if not specified
                rev_dir = reverse_direction.get(motor_id, False)
                # Convert to drive_mode_value (0 or 1)
                drive_mode_value = 1 if rev_dir else 0
                param_dict[motor_id] = drive_mode_value
        else:
            # If reverse_direction is not a dict, apply the same drive mode to all motors
            drive_mode_value = 1 if reverse_direction else 0
            param_dict = {motor_id: drive_mode_value for motor_id in motor_ids}

        logging.debug(f"Setting drive mode for group '{group_name}', reverse_direction={reverse_direction}, param_dict={param_dict}")

        # Disable torque before setting drive mode
        self.torque_off_group(group_name)

        # Sync write drive mode for all motors
        try:
            logging.debug(f"Sync writing drive mode values: {param_dict}")
            self.sync_write_group(group_name, 'drive_mode', param_dict)
            logging.info(f"Drive mode set for group {group_name} with reverse_direction={reverse_direction}")
        except Exception as e:
            logging.error(f"Failed to set drive mode for group {group_name}: {e}")

        # Re-enable torque
        self.torque_on_group(group_name)

    def increment_group_position(self, group_name, increment_degrees):
        """
        Increment the motor positions for a group of motors by a specified number of degrees.
        
        :param group_name: The name of the motor group.
        :param increment_degrees: The number of degrees to increment the motor position, can be an int for all motors or a dict with motor IDs as keys and increments as values.
        """
        motor_ids = self.motor_groups.get(group_name, [])
        if not motor_ids:
            logging.warning(f"No motors found for group '{group_name}'")
            return
        
        logging.info(f"Incrementing motor positions by {increment_degrees} degrees for group '{group_name}'")

        # Ensure the motors are in the correct operating mode
        logging.debug(f"Checking operating mode for motors in group '{group_name}'...")
        operating_modes = self.bulk_read_group(group_name, ['operating_mode'])
        if operating_modes is None:
            logging.error(f"Failed to read operating modes for group '{group_name}'")
            return

        # Check if all motors are in Extended Position Control Mode (mode 4)
        for motor_id, data in operating_modes.items():
            if 'operating_mode' not in data or data['operating_mode'] != 4:
                logging.warning(f"Motor {motor_id} is not in Extended Position Control Mode. Setting mode now.")
                self.set_operating_mode_group(group_name, 'multi_turn')
                break  # Set the mode for the entire group and continue

        # Read current positions
        logging.debug(f"Reading current positions for motors in group '{group_name}'...")
        motor_data = self.bulk_read_group(group_name, ['present_position'])
        
        if motor_data is None:
            logging.error(f"Failed to read motor positions for group '{group_name}'")
            return

        # Create a dictionary for new positions
        new_positions = {}

        # Handle increment_degrees being either an int or a dict
        if isinstance(increment_degrees, dict):
            # Handle per-motor increments from the dictionary
            for motor_id in motor_ids:
                # Ensure motor data contains present_position for each motor
                if motor_id not in motor_data or 'present_position' not in motor_data[motor_id]:
                    logging.error(f"No position data found for motor {motor_id}")
                    continue

                # Get the present_position (raw value)
                current_position = motor_data[motor_id]['present_position']
                if current_position is None:
                    logging.error(f"No position data found for motor {motor_id}")
                    continue

                # Get the increment for this specific motor, or use 0 if not found
                increment = increment_degrees.get(motor_id, 0)

                # Convert the current position to degrees and calculate new position
                current_position_degrees = self.position_to_degrees(current_position)
                new_position_degrees = current_position_degrees + increment

                # Convert back to raw position value
                new_position_value = self.degrees_to_position(new_position_degrees)
                new_positions[motor_id] = new_position_value

                logging.debug(f"Motor {motor_id}: Current Position: {current_position_degrees:.2f}°, New Position: {new_position_degrees:.2f}°")
        else:
            # Handle single int increment for all motors
            for motor_id in motor_ids:
                # Ensure motor data contains present_position for each motor
                if motor_id not in motor_data or 'present_position' not in motor_data[motor_id]:
                    logging.error(f"No position data found for motor {motor_id}")
                    continue

                # Get the present_position (raw value)
                current_position = motor_data[motor_id]['present_position']
                if current_position is None:
                    logging.error(f"No position data found for motor {motor_id}")
                    continue

                # Convert the current position to degrees and calculate new position
                current_position_degrees = self.position_to_degrees(current_position)
                new_position_degrees = current_position_degrees + increment_degrees

                # Convert back to raw position value
                new_position_value = self.degrees_to_position(new_position_degrees)
                new_positions[motor_id] = new_position_value

                logging.debug(f"Motor {motor_id}: Current Position: {current_position_degrees:.2f}°, New Position: {new_position_degrees:.2f}°")

        # Sync write the new positions
        try:
            self.sync_write_group(group_name, 'goal_position', new_positions)
            logging.debug(f"Motor positions for group '{group_name}' incremented.")
        except Exception as e:
            logging.error(f"Failed to increment motor positions for group '{group_name}': {e}")

    def set_position_limits_group(self, group_name, min_degrees=None, max_degrees=None):
        """
        Set the position limits (min and max) for a group of motors, using degrees or raw motor values.
        
        :param group_name: The name of the motor group to set limits for (e.g., 'Hinges').
        :param min_degrees: The minimum allowable position in degrees. If not provided, it will use defaults from config.
        :param max_degrees: The maximum allowable position in degrees. If not provided, it will use defaults from config.
        """
        try:
            # Ensure the motor group exists
            if group_name not in self.motor_groups:
                logging.error(f"Motor group {group_name} not found")
                return
            
            # Load default limits from YAML if not provided
            if min_degrees is None or max_degrees is None:
                logging.info(f"Loading position limits from config for group {group_name}")
                min_degrees = self.config['position_limits'][group_name]['min_degrees']
                max_degrees = self.config['position_limits'][group_name]['max_degrees']

            # Convert degrees to raw motor position values
            min_position = self.degrees_to_position(min_degrees)
            max_position = self.degrees_to_position(max_degrees)

            # Prepare dictionaries to hold min and max position values for each motor
            min_position_dict = {motor_id: min_position for motor_id in self.motor_groups[group_name]}
            max_position_dict = {motor_id: max_position for motor_id in self.motor_groups[group_name]}

            # Write the min and max position limits using sync write
            self.sync_write_group(group_name, 'min_position_limit', min_position_dict)
            self.sync_write_group(group_name, 'max_position_limit', max_position_dict)

            logging.info(f"Position limits set for group {group_name}: min={min_degrees}° (ticks={min_position}), max={max_degrees}° (ticks={max_position})")
        
        except Exception as e:
            logging.error(f"Failed to set position limits for group {group_name}: {e}")

    def set_motor_baud_rate(self, group_name, baud_rate_value):
        """
        Set the baud rate for a group of motors.
        :param group_name: The name of the motor group (from config).
        :param baud_rate_value: The baud rate value to set (e.g., 3 for 1 Mbps).
        """
        motor_ids = self.motor_groups.get(group_name, [])
        if not motor_ids:
            logging.warning(f"No motors found for group '{group_name}'")
            return

        # Sync write baud rate for all motors in the group
        baud_rate_params = {motor_id: baud_rate_value for motor_id in motor_ids}
        self.sync_write_group(group_name, 'baud_rate', baud_rate_params)

        logging.info(f"Baud rate set to {baud_rate_value} for group {group_name}")

    def close(self):
        """
        Closes the communication port and performs cleanup.
        """
        try:
            self.port_handler.closePort()
            logging.info("Port closed successfully.")
        except Exception as e:
            logging.error(f"Failed to close the port: {e}")

    def reboot_motor(self, motor_id):
        """
        Reboot a specific motor and re-enable torque.
        
        :param motor_id: The ID of the motor to reboot.
        :return: True if successful, False if an error occurs.
        """
        try:
            # Send the reboot instruction to the motor
            result, error = self.packet_handler.reboot(self.port_handler, motor_id)
            if result != 0:
                # Log the error if the reboot command fails
                logging.error(f"Failed to reboot motor {motor_id}: {self.packet_handler.getRxPacketError(error)}")
                return False

            logging.info(f"Motor {motor_id} successfully rebooted.")

            return True

        except Exception as e:
            logging.error(f"Error rebooting motor {motor_id}: {e}")
            return False
