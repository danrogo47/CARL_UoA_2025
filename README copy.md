# BIO-INSPIRED MORPHING ROBOT FOR EXTRATERRESTRIAL TERRAIN

This repository contains the code required to run the FLIK robot, a bio-inspired morphing robot designed for extraterrestrial terrain.

## Contents
1. [Installation](#installation)
2. [Usage](#usage)
3. [Configuration File Overview](#configuration-file-overview)
4. [Controller Script Overview](#controller-script-overview)
5. [Dynamixel Control Script Overview](#dynamixel-control-script-overview)
6. [Main Script Overview](#main-script-overview)
7. [Contact](#contact)

---

## Installation

Before running the FLIK robot, ensure the following packages are installed on the Raspberry Pi. An existing venv virtual environment is setup on the Pi 4B as of November 2024. This is located in Documents/VirtualEnvs/Flik and contains all required packges.

### Required Packages

1. **Dynamixel SDK**  
   To install the Dynamixel SDK:  
    ```bash
   `sudo apt-get install dynamixel-sdk`
    ```

2. **PyGame**  
   Install the `pygame` library for PS4 controller support: 
   ```bash 
   Insert: `pip3 install pygame`
    ```

3. **Arduino CLI**  
   Arduino CLI is required for uploading firmware to the Arduino:
   ```bash  
   `curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh`
   ```

4. **YAML**  
   Install the `pyyaml` library for YAML configuration file parsing:  
   ```bash
   Insert: `pip3 install pyyaml`
   ```

---

## Usage

### Flashing the Arduino Board

Before running the `main.py` file, the init.ino script must be compiled and uploaded to the OpenRB-150 board. This can be done directly from the Raspberry Pi using the following steps:

1. **Compile the Arduino Code**  
   Navigate to the directory containing the `init.ino` file:
   ```bash  
   Insert: `cd /home/flik/Documents/Repos/Bio-Inspired-Shape-Shifting-Robot/RPi/Ardinuo/init`
   ```

   Compile the Arduino sketch using the `arduino-cli`:  
   ```bash
   Insert: `arduino-cli compile --fqbn OpenRB-150:samd:OpenRB-150 .`
   ```

2. **Upload the Code to the Arduino**  
   Once compiled, upload the code to the OpenRB-150 board:  
   ```bash
   Insert: `arduino-cli upload -p /dev/ttyACM0 --fqbn OpenRB-150:samd:OpenRB-150 .`
   ```

Once the code is successfully uploaded follow the steps to connect the PS4 controller in [Controller Script Overview](#controller-script-overview)

### Running the Robot

After loading the init.ino, connecting the controller via Bluetooth, and turning the robot on, you can initialise the robot by running the following command:  
```bash
Insert: `python3 main.py`
```

This will cause the robot to stand up so ensure a safe distance is maintained.

---

## Configuration File Overview

The `config.yaml` file centralises key parameters for FLIK's operation. It simplifies changes to logging, motor control, and gaits.

### Key Sections

- **Logging**: Controls logging levels for both file and console outputs.

- **Motor IDs**: Assigns IDs for wheg motors (6) and pivot motors (2).

- **Wheg & Pivot Parameters**: Sets speed limits, initial conditions, and control steps for the wheg and pivot motors.

- **Controller**: Defines the communication parameters, including device path and baud rate.

- **Gaits**: Lists available gaits and their parameters (angles, speeds, delays) for smooth operation.

- **Reporting**: Sets intervals for logging motor positions for real-time monitoring.

- **Velocity Limits**: Controls maximum velocities for motor groups and individual motors.

- **Profile Velocities**: Predefined velocity settings for synchronised control of motor groups.

- **Hard Limits**: Safety parameters for motor velocity and profile velocity limits.

- **Position Limits**: Configures movement boundaries for hinges and whegs.

- **Control Table**: Dynamixel motor register addresses for torque, velocity, position, etc.

- **Motor Groups**: Defines motor groupings for synchronised commands (e.g., all whegs, pivots, left vs right whegs).

### Usage

- **Edit Parameters**: Modify `config.yaml` to adjust logging levels, motor speeds, or gaits.

- **Add Gaits**: New gaits can be added under `gaits` with custom angles, speeds, and timing.

- **Motor ID Updates**: If motor IDs change, update the `motor_ids` section.

---

## Controller Script Overview

This script utilises the `pygame` library to interface with a PS4 controller connected to the Raspberry Pi. It reads inputs from the controller's joysticks, buttons, triggers, and D-pad. Additionally, the script includes debounce logic to prevent rapid sequential button presses from registering, ensuring smooth control in real-time applications.

### Key Functionalities:

- **Debounce Logic**: Prevents rapid button presses within a specified interval (default: 0.25 seconds) to avoid unintended multiple inputs.
- **Connection Monitoring**: Continuously checks if the controller is connected and logs any connection status changes.
- **Input Handling**: Gathers inputs from various controller components such as joysticks, buttons, triggers, and the D-pad.
- **Asyncio Integration**: The input monitoring functions can be used with `asyncio` for continuous input detection without blocking the main program flow.

### Connecting the PS4 Controller to the Raspberry Pi

To connect your PS4 controller to the Raspberry Pi via Bluetooth, follow these steps:

1. Open a terminal on your Raspberry Pi and run:

    ```bash
    sudo bluetoothctl
    ```

2. Inside the `bluetoothctl` prompt, run the following commands:

    ```bash
    power on
    agent on
    default-agent
    scan on
    ```

3. Put the PS4 controller into pairing mode by holding the **Share** button and the **PS** button simultaneously until the light on the controller starts blinking.

4. Once the Raspberry Pi detects the controller, note its MAC address (a format like `XX:XX:XX:XX:XX:XX`) and run the following command:

    ```bash
    pair XX:XX:XX:XX:XX:XX
    ```

5. After pairing, run:

    ```bash
    connect XX:XX:XX:XX:XX:XX
    ```

6. Finally, to trust the device so that it reconnects automatically in the future, use:

    ```bash
    trust XX:XX:XX:XX:XX:XX
    ```

Now, your PS4 controller should be connected to the Raspberry Pi via Bluetooth, ready for input reading.

### Functions:

#### `__init__(self, debounce_time=0.25)`
Initialises the PS4 controller interface:
- Sets up `pygame` and initialises the joystick.
- Defines button indices and configures debounce timing.
- This method also assumes the controller is already connected via Bluetooth.

#### `check_controller_connected(self)`
Checks whether the controller is still connected and logs changes:
- Returns `True` if connected, `False` if disconnected.

#### `get_joystick_input(self)`
Reads joystick input for both left and right sticks:
- Returns X and Y-axis values for both joysticks.

#### `get_button_input(self)`
Retrieves the state of all buttons on the controller with debounce logic:
- Returns a dictionary with button states (pressed or not).
- Logs changes in button states and filters out rapid presses.

#### `get_trigger_input(self)`
Reads the L2 and R2 trigger values:
- Returns the axis values for both triggers (ranges from -1 to 1).

#### `get_dpad_input(self)`
Reads the D-pad (Hat switch) input:
- Returns a dictionary indicating which direction on the D-pad is pressed (up, down, left, right).

#### `close(self)`
Closes the controller interface and quits `pygame`:
- Ensures a proper cleanup of resources when the script ends.

### Asyncio for Continuous Input Monitoring

To continuously monitor inputs from the PS4 controller, you can use `asyncio` to run the input-checking functions without blocking the main loop. This way, the program can perform other tasks while still checking for controller input.

Example usage with `asyncio`:

```python
import asyncio
from ps4_controller import PS4Controller

async def monitor_controller(controller):
    while True:
        joystick_input = controller.get_joystick_input()
        button_input = controller.get_button_input()
        dpad_input = controller.get_dpad_input()
        trigger_input = controller.get_trigger_input()
        
        # Handle the inputs here
        
        await asyncio.sleep(0.01)  # Add a small delay to reduce CPU usage

controller = PS4Controller()

async def main():
    await monitor_controller(controller)

asyncio.run(main())

```

---

## Dynamixel Control Script Overview

This script utilises the `dynamixel_sdk` library to control Dynamixel motors connected to a Raspberry Pi via an Open RB150 motor controller board. It leverages `sync write` and `bulk write` commands to control multiple motors simultaneously, streamlining movement and command execution for the motors.

### Key Functionalities:

- **Configuration from YAML**: Motor parameters and control tables are loaded from a configuration YAML file (`config.yaml`), allowing easy adjustment of motor settings.
- **Motor Grouping**: Motors can be grouped for synchronised control, making it easier to manage different sets of motors (e.g., whegs, pivots).
- **Sync Write and Bulk Write**: Commands can be sent to multiple motors simultaneously, improving performance and efficiency.
- **Error Handling and Logging**: Detailed logging and error handling ensure smooth operation, with clear error messages when issues occur.

### Connecting Dynamixel Motors to the Raspberry Pi

To use this script, the Raspberry Pi needs to be connected to the Dynamixel motors via an Open RB150 motor controller board. The motors are controlled through the `dynamixel_sdk`, which communicates over a serial port.

### Functions:

#### `__init__(self, config_path='config.yaml', device_name=None, baudrate=None, protocol_version=2.0)`
Initialises the controller, loads the configuration file, sets up motor groups, and opens the port for communication.

- Loads the configuration from the provided YAML file.
- Sets up the serial port for communication using the specified `device_name` and `baudrate`.
- Initialises the control table and motor groups for easier motor management.

#### `load_config(self, config_path)`
Loads the configuration from a YAML file. This function ensures that motor groups, control table settings, and other parameters are correctly set up.

#### `open_port(self)`
Opens the serial port for communication with the motors, ensuring the connection is established with the correct baud rate.

#### `create_motor_group(self, group_name, motor_ids)`
Creates a group of motors for easier simultaneous control (e.g., all whegs, pivots). Motor groups allow synchronised commands to be sent efficiently.

#### `setup_motor_groups(self)`
Sets up motor groups from the YAML configuration, mapping motor names to IDs and organising them into functional groups for control.

#### `load_control_table(self)`
Loads the control table from the configuration file. The control table contains the necessary addresses and lengths for commands like setting motor positions and velocities.

#### `sync_write_group(self, group_name, parameter_name, param_dict)`
Sends a sync write command to a group of motors. This function is used to control parameters such as goal position and velocity for multiple motors simultaneously.

#### `bulk_read_group(self, group_name, parameters)`
Executes a bulk read command to retrieve data from a group of motors. This is useful for reading multiple parameters (e.g., present position, velocity) from multiple motors in a single operation.

#### `set_operating_mode_group(self, group_name, mode)`
Sets the operating mode (e.g., position control, velocity control, multi-turn mode) for all motors in a group. It ensures that the correct mode is applied before setting motor positions or velocities.

#### `set_status_return_level(self, group_name, level=1)`
Sets the status return level for a group of motors. The status return level determines whether the motors return status packets after receiving commands.

#### `set_group_velocity_limit(self, group_name)`
Sets the velocity limit for a group of motors, ensuring that the motors do not exceed the hard velocity limit specified in the configuration.

#### `set_position_group(self, group_name, positions)`
Sets the target positions (in degrees) for a group of motors. This function can either apply the same position to all motors in the group or set individual positions for each motor.

#### `increment_group_position(self, group_name, increment_degrees)`
Increments the motor positions for a group by a specified number of degrees. This function is useful for making relative adjustments to motor positions.

#### `set_drive_mode_group(self, group_name, reverse_direction=False)`
Sets the drive mode (e.g., normal or reverse direction) for a group of motors. This allows for custom control of motor direction, either synchronously for all motors or individually for each motor.

#### `set_position_limits_group(self, group_name, min_degrees=None, max_degrees=None)`
Sets the position limits (minimum and maximum) for a group of motors, ensuring that they do not exceed specified angular ranges during operation.

#### `reboot_motor(self, motor_id)`
Reboots a specific motor and re-enables torque. This function is useful for recovering from errors or resetting motors.

#### `close(self)`
Closes the serial communication port and performs cleanup.

---

### Usage

1. **Initialising the Controller**:
   Create an instance of the `DynamixelController` class to initialise the system and load the configuration.

   Example:
   ```python
   controller = DynamixelController(config_path='config.yaml', device_name='/dev/ttyUSB0', baudrate=1000000)
   ```

2. **Controlling Motors**
    Use the provided methods to control motor groups, set positions, velocities and operating modes.

    Example:
    ```python
    contoller.set_position_group('Wheg_Group', positions=180)
    ```

3. **Closing the Controller**
    Always call 'controller.close()' to ensure that the serial port is closed after operation.

    Example:
    ```Python
    controller.close()
    ```

---

## Main Script Overview

This script implements the main logic for converting PS4 controller inputs into Dynamixel motor commands. It utilises asynchronous tasks to handle different functions, such as gait control, pivot control, and motor state reporting, all while reading input from the PS4 controller and converting it into commands for the Dynamixel motors.

### Key Features:

- **Controller Inputs**: Maps the PS4 controller inputs (joysticks, buttons, D-pad, triggers) to control the robot's whegs (wheeled legs) and pivot motors.
- **Asynchronous Tasks**: Multiple tasks run concurrently, including input checking, gait execution, and robot state reporting to ensure smooth, responsive control.
- **Gait Control**: Implements multiple gaits that can be switched on the fly, allowing for different movement patterns of the robot's whegs.
- **Logging**: Logs critical robot states such as motor positions, velocities, loads, and controller inputs. It also records hardware errors and automatically logs actions like gait changes and emergency stops.

### Dependencies:

- `DynamixelController` class from `dynamixel_control.py`
- `PS4Controller` class from `controller.py`
- Dynamixel SDK
- Initialisation file uploaded to the Open RB-150 Board

### Functions:

#### `FLIKRobot.__init__(self)`
Initialises the robot's controller and motor systems:

- Loads the configuration file (`config.yaml`).
- Sets up logging to both file and console.
- Initialises the PS4 controller and Dynamixel motor controller.
- Configures the whegs and pivot motors based on the YAML configuration.

#### `setup_logging(self)`
Sets up the logging system for both console output and file logging. A log file is generated based on the current date and time. Also creates a CSV file to log motor positions.

#### `setup_whegs(self)` and `setup_pivots(self)`
Configures the wheg motors and pivot motors based on the YAML settings, including setting direction modes and position limits.

#### `adjust_wheg_rpm(self, trigger_value)`
Adjusts the speed of the whegs based on the right trigger input, with a smooth transition to the target RPM.

#### `log(self, motor_positions, l2_trigger, r2_trigger, button_states, dpad_input)`
Logs the current state of the robot, including motor positions, RPMs, and controller inputs.

#### `control_pivots_with_dpad(self)`
Controls the front and rear pivot motors using D-pad inputs from the PS4 controller.

#### `gait_init_1(self)` to `gait_init_4(self)`
Initialises the different gaits by configuring motor speeds, positions, and modes.

#### `gait_1(self)` to `gait_4(self)`
Executes the different gaits by controlling the movement patterns of the whegs.

#### `reverse_direction(self)`
Reverses the direction of the whegs based on the "Share" button input from the controller.

#### `async_emergency_stop(self)`
Asynchronously stops all motors if the "Circle" button is pressed, indicating an emergency stop.

#### `check_inputs(self)`
Monitors the PS4 controller inputs asynchronously, checking for gait changes, direction changes, or emergency stops.

#### `execute_gait(self)`
Executes the current gait asynchronously, with a mechanism to switch between different gaits.

#### `report_states(self)`
Asynchronously reports the robot's state, including motor positions, velocities, loads, and hardware errors, at a specified interval.

#### `write_to_csv(self)`
Logs motor positions, velocities, and loads into a CSV file at a specified interval for later analysis.

#### `main_loop(self)`
The main asynchronous loop that runs the different tasks, such as checking inputs, controlling gaits, and reporting states.

### Usage

1. **Initialising the Robot**:
   Create an instance of the `FLIKRobot` class, which loads the configuration and sets up the controller and motors.

   Example:
    ```Python
    robot = FLIKRobot()
    ```
    
---

## Contact

### Tyson Baker - Tysonbaker2072@gmail.com

You can ask someone else from the group, but they probably won't know anything about the code.