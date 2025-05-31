import logging
from dynamixel_control import DynamixelController
import time

# Initialize logging for console output
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s %(levelname)s: %(message)s')

def test_dynamixel_controller():
    """
    Test the DynamixelController with basic motor group operations.
    """
    try:
        # Initialize the controller
        logging.debug("Test Case: Initialize DynamixelController")
        controller = DynamixelController(config_path='config.yaml')
        logging.info("DynamixelController initialized successfully.")

        # Open port and set baudrate
        logging.debug("Test Case: Open port and set baudrate")
        controller.open_port()
        logging.info(f"Port {controller.device_name} opened with baudrate {controller.baudrate}.")

        # Test torque off and on for the 'Wheg_Group'
        logging.debug("Test Case: Torque OFF for Wheg_Group")
        controller.torque_off_group('Wheg_Group')
        logging.info("Torque disabled for Wheg_Group.")
        time.sleep(1)

        logging.debug("Test Case: Torque ON for Wheg_Group")
        controller.torque_on_group('Wheg_Group')
        logging.info("Torque enabled for Wheg_Group.")
        time.sleep(1)

        # Sync write to set initial positions
        logging.debug("Test Case: Sync Write - Set initial motor positions")
        initial_positions = {motor_id: 2048 for motor_id in controller.motor_groups['Wheg_Group']}
        controller.sync_write_group('Wheg_Group', 'position_goal', initial_positions)
        logging.info("Sync write (position_goal) executed successfully.")
        time.sleep(2)

        # Bulk read motor positions in degrees
        logging.debug("Test Case: Bulk Read - Read motor positions (in degrees)")
        motor_positions = controller.bulk_read_group('Wheg_Group', ['present_position'])
        for motor_id, data in motor_positions.items():
            logging.info(f"Motor {motor_id} Position (Degrees): {data['position_degrees']:.2f}")

        # Set operating mode to velocity control
        logging.debug("Test Case: Set operating mode to velocity control")
        controller.set_operating_mode_group('Wheg_Group', 'velocity')
        logging.info("Operating mode set to 'velocity' successfully.")

        # Set velocity limits
        logging.debug("Test Case: Set velocity limit for Wheg_Group")
        controller.set_group_velocity_limit('Wheg_Group')
        logging.info("Velocity limit set successfully.")

        # Set profile velocity
        logging.debug("Test Case: Set profile velocity for Wheg_Group")
        controller.set_group_profile_velocity('Wheg_Group')
        logging.info("Profile velocity set successfully.")

        # Set positions to 180 degrees
        logging.debug("Test Case: Set target positions")
        positions = {motor_id: 180.0 for motor_id in controller.motor_groups['Wheg_Group']}
        controller.set_position_group('Wheg_Group', positions)
        logging.info(f"Positions set to {positions}")
        time.sleep(2)

        # Bulk read current positions
        logging.debug("Test Case: Bulk Read - Get current positions (in degrees)")
        motor_positions = controller.bulk_read_group('Wheg_Group', ['present_position'])
        for motor_id, data in motor_positions.items():
            logging.info(f"Motor {motor_id} Position (Degrees): {data['position_degrees']:.2f}")

        # Bulk read velocities with torque off
        logging.debug("Test Case: Torque OFF and Bulk Read - Get current velocities")
        controller.torque_off_group('Wheg_Group')
        motor_data = controller.bulk_read_group('Wheg_Group', ['present_velocity'])
        for motor_id, data in motor_data.items():
            logging.info(f"Motor {motor_id} Velocity: {data.get('present_velocity', 0)}")
        controller.torque_on_group('Wheg_Group')
        logging.info("Torque re-enabled for Wheg_Group.")

        # Set and test drive mode
        logging.debug("Test Case: Set drive mode for Right_Whegs")
        controller.set_drive_mode_group('Right_Whegs', reverse_direction=True)
        motor_data = controller.bulk_read_group('Right_Whegs', ['drive_mode'])
        for motor_id, data in motor_data.items():
            drive_mode = data.get('drive_mode', None)
            if drive_mode == 1:
                logging.info(f"Motor {motor_id} is set to reverse direction.")
            else:
                logging.error(f"Motor {motor_id} drive mode is not correctly set to reverse.")
        controller.set_drive_mode_group('Right_Whegs', reverse_direction=False)

        logging.info("All tests completed successfully.")
        return controller

    except Exception as e:
        logging.error(f"Test failed: {e}")
        return None

def test_increment_motor_position(controller):
    """
    Test to increment motor positions by 90 degrees in extended position mode.
    """
    try:
        logging.debug("Test Case: Increment motor positions by 90 degrees")
        controller.increment_motor_position_by_degrees('Wheg_Group', 90)
        motor_data = controller.bulk_read_group('Wheg_Group', ['present_position'])
        for motor_id, data in motor_data.items():
            position_degrees = controller.position_to_degrees(data.get('present_position', 0))
            logging.info(f"Motor {motor_id} Position after increment (Degrees): {position_degrees}")

    except Exception as e:
        logging.error(f"Increment motor position test failed: {e}")

def test_bulk_read():
    """
    Test the bulk read functionality by reading present positions, velocities, and operating modes of motors.
    """
    try:
        # Initialize the DynamixelController with the config file path
        logging.debug("Test Case: Initialize DynamixelController")
        controller = DynamixelController(config_path='config.yaml')
        logging.info("DynamixelController initialized successfully.")

        # Open port and set baudrate
        logging.debug("Test Case: Open port and set baudrate")
        controller.open_port()
        logging.info(f"Port {controller.device_name} opened with baudrate {controller.baudrate}.")
        
        # Define motor groups to be tested
        test_groups = ['Wheg_Group', 'Pivot_Group', 'Right_Whegs']

        for group in test_groups:
            logging.debug(f"Test Case: Bulk read for group: {group}")
            
            # Test reading present position
            try:
                motor_data = controller.bulk_read_group(group, ['present_position'])
                if motor_data is None:
                    logging.error(f"Bulk read failed for group '{group}' (present_position)")
                else:
                    for motor_id, data in motor_data.items():
                        position_degrees = data.get('position_degrees', None)
                        if position_degrees is not None:
                            logging.info(f"Motor {motor_id} Present Position (Degrees): {position_degrees:.2f}")
                        else:
                            logging.error(f"Motor {motor_id}: Failed to read position.")
            except Exception as e:
                logging.error(f"Error reading present position for group '{group}': {e}")

            # Test reading present velocity
            try:
                motor_data = controller.bulk_read_group(group, ['present_velocity'])
                if motor_data is None:
                    logging.error(f"Bulk read failed for group '{group}' (present_velocity)")
                else:
                    for motor_id, data in motor_data.items():
                        velocity = data.get('present_velocity', None)
                        if velocity is not None:
                            logging.info(f"Motor {motor_id} Present Velocity: {velocity}")
                        else:
                            logging.error(f"Motor {motor_id}: Failed to read velocity.")
            except Exception as e:
                logging.error(f"Error reading present velocity for group '{group}': {e}")

            # Test reading operating mode
            try:
                motor_data = controller.bulk_read_group(group, ['operating_mode'])
                if motor_data is None:
                    logging.error(f"Bulk read failed for group '{group}' (operating_mode)")
                else:
                    for motor_id, data in motor_data.items():
                        mode = data.get('operating_mode', None)
                        if mode is not None:
                            logging.info(f"Motor {motor_id} Operating Mode: {mode}")
                        else:
                            logging.error(f"Motor {motor_id}: Failed to read operating mode.")
            except Exception as e:
                logging.error(f"Error reading operating mode for group '{group}': {e}")

    except Exception as e:
        logging.error(f"Test failed: {e}")

def test_bulk_read_with_torque_off():
    """
    Test the bulk read functionality with torque disabled for a group of motors.
    """
    try:
        # Initialize the DynamixelController with the config file path
        logging.debug("Test Case: Initialize DynamixelController")
        controller = DynamixelController(config_path='config.yaml')
        logging.info("DynamixelController initialized successfully.")

        # Open port and set baudrate
        logging.debug("Test Case: Open port and set baudrate")
        controller.open_port()
        logging.info(f"Port {controller.device_name} opened with baudrate {controller.baudrate}.")

        # Define the motor group for the test
        group_name = 'Wheg_Group'

        # Step 1: Disable torque for the motor group
        logging.debug(f"Test Case: Disable torque for group: {group_name}")
        controller.torque_off_group(group_name)
        logging.info(f"Torque disabled for group {group_name}.")

        # Step 2: Perform bulk read for present position with torque off
        logging.debug(f"Test Case: Bulk read for group: {group_name} (present_position) with torque off")
        motor_data = controller.bulk_read_group(group_name, ['present_position'])
        if motor_data is None:
            logging.error(f"Bulk read failed for group '{group_name}' (present_position) with torque off")
        else:
            for motor_id, data in motor_data.items():
                position_degrees = data.get('position_degrees', None)
                if position_degrees is not None:
                    logging.info(f"Motor {motor_id} Present Position (Degrees) with torque off: {position_degrees:.2f}")
                else:
                    logging.error(f"Motor {motor_id}: Failed to read position with torque off.")

        # Step 3: Perform bulk read for present velocity with torque off
        logging.debug(f"Test Case: Bulk read for group: {group_name} (present_velocity) with torque off")
        motor_data = controller.bulk_read_group(group_name, ['present_velocity'])
        if motor_data is None:
            logging.error(f"Bulk read failed for group '{group_name}' (present_velocity) with torque off")
        else:
            for motor_id, data in motor_data.items():
                velocity = data.get('present_velocity', None)
                if velocity is not None:
                    logging.info(f"Motor {motor_id} Present Velocity with torque off: {velocity}")
                else:
                    logging.error(f"Motor {motor_id}: Failed to read velocity with torque off.")

        # Step 4: Perform bulk read for operating mode with torque off
        logging.debug(f"Test Case: Bulk read for group: {group_name} (operating_mode) with torque off")
        motor_data = controller.bulk_read_group(group_name, ['operating_mode'])
        if motor_data is None:
            logging.error(f"Bulk read failed for group '{group_name}' (operating_mode) with torque off")
        else:
            for motor_id, data in motor_data.items():
                mode = data.get('operating_mode', None)
                if mode is not None:
                    logging.info(f"Motor {motor_id} Operating Mode with torque off: {mode}")
                else:
                    logging.error(f"Motor {motor_id}: Failed to read operating mode with torque off.")

        # Step 5: Re-enable torque for the motor group
        logging.debug(f"Test Case: Re-enable torque for group: {group_name}")
        controller.torque_on_group(group_name)
        logging.info(f"Torque re-enabled for group {group_name}.")

    except Exception as e:
        logging.error(f"Test failed: {e}")

def test_set_operating_mode_group(controller, group_name, mode):
    logging.debug(f"Test Case: Set operating mode for group {group_name} to {mode}")
    try:
        controller.set_operating_mode_group(group_name, mode)
        logging.info(f"Operating mode for group {group_name} set to {mode} successfully.")
    except Exception as e:
        logging.error(f"Failed to set operating mode for group {group_name}: {e}")

def test_set_group_velocity_limit(controller, group_name):
    logging.debug(f"Test Case: Set velocity limit for group {group_name}")
    try:
        controller.set_group_velocity_limit(group_name)
        logging.info(f"Velocity limit for group {group_name} set successfully.")
    except Exception as e:
        logging.error(f"Failed to set velocity limit for group {group_name}: {e}")

def test_set_group_profile_velocity(controller, group_name, profile_velocity=None):
    logging.debug(f"Test Case: Set profile velocity for group {group_name}")
    try:
        controller.set_group_profile_velocity(group_name, profile_velocity)
        logging.info(f"Profile velocity for group {group_name} set to {profile_velocity} successfully.")
    except Exception as e:
        logging.error(f"Failed to set profile velocity for group {group_name}: {e}")

def test_torque_off_group(controller, group_name):
    logging.debug(f"Test Case: Disable torque for group {group_name}")
    try:
        controller.torque_off_group(group_name)
        logging.info(f"Torque disabled for group {group_name}.")
    except Exception as e:
        logging.error(f"Failed to disable torque for group {group_name}: {e}")

def test_torque_on_group(controller, group_name):
    logging.debug(f"Test Case: Enable torque for group {group_name}")
    try:
        controller.torque_on_group(group_name)
        logging.info(f"Torque enabled for group {group_name}.")
    except Exception as e:
        logging.error(f"Failed to enable torque for group {group_name}: {e}")

def test_set_position_group(controller, group_name, positions_dict):
    logging.debug(f"Test Case: Set position for group {group_name}")
    try:
        controller.set_position_group(group_name, positions_dict)
        logging.info(f"Positions for group {group_name} set successfully: {positions_dict}")
    except Exception as e:
        logging.error(f"Failed to set positions for group {group_name}: {e}")

def test_set_velocity_group(controller, group_name, velocities_dict):
    logging.debug(f"Test Case: Set velocity for group {group_name}")
    try:
        controller.set_velocity_group(group_name, velocities_dict)
        logging.info(f"Velocities for group {group_name} set successfully: {velocities_dict}")
    except Exception as e:
        logging.error(f"Failed to set velocities for group {group_name}: {e}")

def test_set_status_return_level(controller, group_name, level=2):
    logging.debug(f"Test Case: Set status return level for group {group_name} to {level}")
    try:
        controller.set_status_return_level_group(group_name, level)
        logging.info(f"Status return level set to {level} for group {group_name}.")
    except Exception as e:
        logging.error(f"Failed to set status return level for group {group_name}: {e}")

def test_baud_rate(controller, group_name):
    logging.debug(f"Test Case: Test communication with group '{group_name}' at new baud rate")
    try:
        # Perform a bulk read to check communication
        motor_data = controller.bulk_read_group(group_name, ['present_position'])
        if motor_data is None:
            logging.error(f"Failed to communicate with motors in group '{group_name}'")
        else:
            logging.info(f"Successfully communicated with motors in group '{group_name}'")
    except Exception as e:
        logging.error(f"Test failed for group '{group_name}': {e}")

def test_set_drive_mode_group(controller, group_name, reverse_direction):
    logging.debug(f"Test Case: Set drive mode for group {group_name} with reverse_direction={reverse_direction}")

    try:
        # Set the drive mode
        controller.set_drive_mode_group(group_name, reverse_direction)

        # Bulk read the drive mode to verify
        motor_data = controller.bulk_read_group(group_name, ['drive_mode'])
        if motor_data is None:
            logging.error(f"Failed to read drive mode for group '{group_name}'")
            return

        # Verify the drive mode
        drive_mode_value = 1 if reverse_direction else 0
        for motor_id, data in motor_data.items():
            current_drive_mode = data.get('drive_mode', None)
            logging.debug(f"Motor {motor_id} current drive mode read from bulk read: {current_drive_mode}")
            if current_drive_mode != drive_mode_value:
                logging.error(f"Motor {motor_id} drive mode is not correctly set to {'reverse' if reverse_direction else 'normal'}")
            else:
                logging.info(f"Motor {motor_id} drive mode correctly set to {'reverse' if reverse_direction else 'normal'}")
    except Exception as e:
        logging.error(f"Failed to set drive mode for group {group_name}: {e}")

def test_increment_motor_position_by_degrees(controller, group_name, increment_degrees):
    logging.debug(f"Test Case: Increment motor position for group {group_name} by {increment_degrees} degrees")

    try:
        # Bulk read current positions (which returns position in degrees)
        motor_data = controller.bulk_read_group(group_name, ['present_position'])
        if motor_data is None:
            logging.error(f"Failed to read motor positions for group '{group_name}'")
            return

        # Create a dictionary for new positions in degrees
        new_positions_dict = {}
        for motor_id, data in motor_data.items():
            current_position_degrees = data.get('position_degrees')
            if current_position_degrees is None:
                logging.error(f"No position data found for motor {motor_id}")
                logging.debug(f"Raw data received for motor {motor_id}: {data}")
                continue

            new_position_degrees = current_position_degrees + increment_degrees
            new_positions_dict[motor_id] = new_position_degrees

        # Call the controller function to write the new positions
        controller.set_position_group(group_name, new_positions_dict)

        logging.info(f"Motor positions for group '{group_name}' incremented by {increment_degrees} degrees.")
    except Exception as e:
        logging.error(f"Failed to increment motor position for group {group_name}: {e}")


def test_set_position_limits_group(controller, group_name, min_degrees=None, max_degrees=None):
    logging.debug(f"Test Case: Set position limits for group {group_name}")
    try:
        controller.set_position_limits_group(group_name, min_degrees, max_degrees)
        logging.info(f"Position limits set for group {group_name}: min={min_degrees}, max={max_degrees}")
    except Exception as e:
        logging.error(f"Failed to set position limits for group {group_name}: {e}")

def run_all_tests():
    try:
        # Initialize the DynamixelController
        logging.debug("Test Case: Initialize DynamixelController")
        controller = DynamixelController(config_path='config.yaml')
        logging.info("DynamixelController initialized successfully.")

        # Define motor group for testing
        group_name = 'Wheg_Group'
        positions_dict = {1: 180.0, 2: 180.0, 3: 180.0}  # Test with example positions
        velocities_dict = {1: 50, 2: 50, 3: 50}  # Test with example velocities

        # Run status return level test before running other tests
        test_set_status_return_level(controller, group_name, level=2)

        # Run each test
        test_baud_rate(controller, 'All_Motors')
        test_set_operating_mode_group(controller, group_name, 'position')
        test_set_group_velocity_limit(controller, group_name)
        test_set_group_profile_velocity(controller, group_name, 60)
        test_torque_off_group(controller, group_name)
        test_torque_on_group(controller, group_name)
        test_set_position_group(controller, group_name, positions_dict)
        test_set_velocity_group(controller, group_name, velocities_dict)
        test_set_drive_mode_group(controller, group_name, reverse_direction=True)
        test_increment_motor_position_by_degrees(controller, group_name, 90)
        test_set_position_limits_group(controller, group_name, min_degrees=0, max_degrees=360)

    except Exception as e:
        logging.error(f"Test execution failed: {e}")

def test_pivot_motors(dynamixel):
    dynamixel.torque_on_group('Pivot_Group')
    dynamixel.set_operating_mode_group('Pivot_Group', 'position')
    pivot_positions = {
        7: dynamixel.degrees_to_position(90),
        8: dynamixel.degrees_to_position(90)
    }
    dynamixel.set_position_group('Pivot_Group', 165)

def test_pivot_motors_2(dynamixel):
    dynamixel.torque_on_group('Pivot_Group')
    dynamixel.set_operating_mode_group('Pivot_Group', 'position')
    pivot_positions = {
        7: dynamixel.degrees_to_position(90),
        8: dynamixel.degrees_to_position(90)
    }
    dynamixel.set_position_group('Pivot_Group', pivot_positions)

def test_wheg_motors(dynamixel):
    dynamixel.torque_on_group('Wheg_Group')
    dynamixel.set_operating_mode_group('Wheg_Group', 'position')
    pivot_positions = {
        1: 90,
        2: 90,
        3: 90,
        4: 90,
        5: 90,
        6: 90
    }
    dynamixel.set_position_group('Wheg_Group', pivot_positions)

if __name__ == "__main__":
    dynamixel = DynamixelController()
    test_pivot_motors(dynamixel)
    test_pivot_motors_2(dynamixel)
    test_wheg_motors(dynamixel)