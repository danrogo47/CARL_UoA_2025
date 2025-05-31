""" This script uses the pygame library to interface with a PS4 controller connected to the Raspberry Pi. It reads the joystick axes, buttons, triggers, and D-pad inputs. The script also includes a debounce logic to ignore rapid sequential button presses within a specified time interval. """
# External imports
import pygame
import logging
import time

class PS4Controller:
    def __init__(self, debounce_time=0.1):
        logging.info("Initializing PS4Controller")

        # Initialize pygame
        pygame.init()
        logging.debug("Pygame initialized")

        try:
            # Initialize the joystick (assuming it's the first one connected)
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            logging.debug("Joystick initialized")
        except pygame.error as e:
            logging.error("PS4 Controller not detected! Please connect the controller.")
            raise e

        # Get the number of buttons on the controller
        self.num_buttons = self.joystick.get_numbuttons()
        logging.debug(f"Number of buttons on the controller: {self.num_buttons}")

        # Button indices
        self.buttons = {
            'square': 3,  # Square button
            'x': 0,       # X button
            'circle': 1,  # Circle button
            'triangle': 2,  # Triangle button
            'l1': 4,      # L1 button
            'r1': 5,      # R1 button
            'l2': 6,      # L2 trigger (binary)
            'r2': 7,      # R2 trigger (binary)
            'share': 8,   # Share button
            'options': 9,  # Options button
            'l3': 10,     # Left joystick button
            'r3': 11,     # Right joystick button
            'ps': 12,     # PS button
        }

        # D-Pad (Hat switch) index
        self.dpad_index = 0

        # Debounce time in seconds
        self.debounce_time = debounce_time
        self.last_button_press_time = {}


    def check_controller_connected(self):
        """Checks if the PS4 controller is still connected and logs only if the connection status changes."""
        try:
            pygame.event.pump()  # Refresh events to check connection
            if not self.joystick.get_init():
                logging.warning("Controller disconnected!")
                return False
            return True
        except pygame.error:
            logging.error("Controller connection lost!")
            return False

    def get_joystick_input(self):
        """Returns joystick axes input and logs only if controller is disconnected."""
        if not self.check_controller_connected():
            logging.error("Controller is disconnected.")
            return None # If controller is disconnected, return None, use this to break the loop in the main program

        pygame.event.pump()
        x_axis_left = self.joystick.get_axis(0)  # Left joystick X-axis
        y_axis_left = self.joystick.get_axis(1)  # Left joystick Y-axis
        x_axis_right = self.joystick.get_axis(3)  # Right joystick X-axis
        y_axis_right = self.joystick.get_axis(4)  # Right joystick Y-axis
        
        return x_axis_left, y_axis_left, x_axis_right, y_axis_right

    def get_button_input(self):
        """Returns button input and only logs changes in button states."""
        if not self.check_controller_connected():
            logging.error("Controller is disconnected.")
            return None 

        pygame.event.pump()
        button_states = {}
        current_time = time.time()

        for button, index in self.buttons.items():
            if index < self.num_buttons:
                is_pressed = self.joystick.get_button(index)
                # Debounce logic: Ignore rapid sequential presses within debounce time
                if is_pressed:
                    if button not in self.last_button_press_time or \
                            (current_time - self.last_button_press_time[button]) > self.debounce_time:
                        self.last_button_press_time[button] = current_time
                        button_states[button] = True
                        logging.debug(f"Button {button} (index {index}) pressed.")
                    else:
                        button_states[button] = False  # Ignore press (debounced)
                        logging.debug(f"Ignoring rapid press of button {button} (index {index})")
                else:
                    button_states[button] = False
            else:
                button_states[button] = None  # Button doesn't exist
                logging.warning(f"Button {button} (index {index}) does not exist on this controller")
        return button_states

    def get_trigger_input(self):
        """Returns trigger input and only logs connection errors."""
        if not self.check_controller_connected():
            logging.error("Controller is disconnected.")
            return None

        pygame.event.pump()
        l2_trigger = self.joystick.get_axis(2)  # L2 trigger (corrected)
        r2_trigger = self.joystick.get_axis(5)  # R2 trigger (corrected)

        return l2_trigger, r2_trigger

    def get_dpad_input(self):
        """Returns D-pad input and logs only connection errors."""
        if not self.check_controller_connected():
            logging.error("Controller is disconnected.")
            return None

        pygame.event.pump()
        dpad_state = self.joystick.get_hat(self.dpad_index)
        
        dpad_input = {
            'dpad_up': dpad_state[1] == 1,   # Y-axis positive
            'dpad_down': dpad_state[1] == -1,  # Y-axis negative
            'dpad_left': dpad_state[0] == -1,  # X-axis negative
            'dpad_right': dpad_state[0] == 1   # X-axis positive
        }

        return dpad_input

    def close(self):
        """Closes the PS4Controller and logs the event."""
        logging.info("Closing PS4Controller and quitting pygame")
        pygame.quit()
