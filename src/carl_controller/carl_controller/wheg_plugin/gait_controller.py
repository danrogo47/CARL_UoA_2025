import logging
import time
from math import pi

class GaitController():
    def __init__(self, config):
        self.config = config
        
    def setup_variables(self):
        # Motor and pivot configurations from the YAML file
        self.WHEGS = self.config['motor_ids']['whegs']
        self.MAX_RPM = self.config['wheg_parameters']['max_rpm']
        self.MIN_RPM = self.config['wheg_parameters']['min_rpm']
        self.SMOOTHNESS = self.config['wheg_parameters']['smoothness']
        self.wheg_rpm = self.config['wheg_parameters']['min_rpm']
        self.current_gait_index = 0
        self.next_gait_index = 0
        self.SHUT_DOWN = False
        self.total_gaits = len(self.config['gaits'])
        self.reboot_requested = False
        self.report_timer = time.time()
        self.gait_change_requested = True
        # Gait parameters
        self.odd_even = 0
        self.gait_parameters = {}
        self.gait2_params = self.config['gaits'].get('gait_2', {})
        self.gait3_params = self.config['gaits'].get('gait_3', {})
        self.gait4_params = self.config['gaits'].get('gait_4', {})
        # Change these positions to a gait1 parameter set
        self.positions = { 1: self.gait4_params['low_pos'], 2: self.gait4_params['low_pos'], 3: self.gait4_params['low_pos'], 4: self.gait4_params['low_pos'], 5: self.gait4_params['low_pos'], 6: self.gait4_params['low_pos'] }
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
        self.velocities = {}
        self.increments = {}
        self.button_states = {}
        self.dpad_inputs = {}
        self.velocity = 0
        self.joystick_inputs = 0
        self.turn_mode_requested = False
        self.turn_mode_active = False
        self.turn_mode_deactivate = False
        self.current_side = "Right"
     
    def init_turn_mode(self):
        logging.info("Initialising Turn Mode")
        self.gait_change_requested = False  # Reset the request flag
        # Update the min and max RPM for this gait:
        self.MIN_RPM = self.gait4_params['min_rpm']
        self.MAX_RPM = self.gait4_params['max_rpm']
        self.SMOOTHNESS = self.gait4_params['smoothness']
        self.odd_even = 0
        self.wheg_rpm = 0
        self.TOLERANCE = self.gait4_params['tolerance']
        self.positions = { 1: self.gait4_params['low_pos'], 2: self.gait4_params['low_pos'], 3: self.gait4_params['low_pos'], 4: self.gait4_params['low_pos'], 5: self.gait4_params['low_pos'], 6: self.gait4_params['low_pos'] }
        wait_time = 3
        logging.info(f"Initialised turn mode, waiting for {wait_time} seconds")
        self.turn_mode_active = True
        self.turn_mode_requested = False
        return
    
    def set_shutdown(self, shutdown):
        """Inform gaits of change in shutdown status."""
        self.SHUT_DOWN  = shutdown
        
    def execute_gait_change(self):
        """Execute the current gait change."""
        if not self.SHUT_DOWN:
            
            # Check if a gait change has been requested
            if self.gait_change_requested:
                # Initialize the new gait
                init_gait_function = self.gait_init_methods[self.next_gait_index]                
                # Update current gait index
                self.current_gait_index = self.next_gait_index
                self.gait_change_requested = False
                logging.info(f"New gait {self.current_gait_index + 1} is now active.")
                
        else:
            logging.debug("Emergency stop activated, gait change paused.")
        
    # Define the initialization for each gait (for whegs only, pivots are disabled)
    def gait_init_1(self):
        logging.info("Initialising Gait 1")
        self.gait_change_requested = False  # Reset the request flag
    
        self.positions = { 1: self.gait4_params['high_pos'], 2: self.gait4_params['high_pos'], 3: self.gait4_params['high_pos'], 4: self.gait4_params['high_pos'], 5: self.gait4_params['high_pos'], 6: self.gait4_params['high_pos'] }
        return

    def gait_init_2(self):
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
        wait_time = 0.5
        logging.info(f"Initialised Gait 2, waiting for {wait_time} seconds")
        return

    def gait_init_3(self):      
        logging.info("Initialsing Gait 3")
        self.gait_change_requested = False  # Reset the request flag
        # Update the min, max and smoothness for this gait
        self.MIN_RPM = self.gait3_params['min_rpm']
        self.MAX_RPM = self.gait3_params['max_rpm']
        self.SMOOTHNESS = self.gait3_params['smoothness']
        self.TOLERANCE = self.gait3_params['tolerance']
        self.wheg_rpm = 0
        self.odd_even = 0
        self.positions = { 1: self.gait3_params['high_pos'], 2: self.gait3_params['low_pos'], 3: self.gait3_params['low_pos'], 4: self.gait3_params['high_pos'], 5: self.gait3_params['low_pos'], 6: self.gait3_params['low_pos']}
        wait_time = 0.5
        logging.info(f"Initialised Gait 3, waiting for {wait_time} seconds")
        return

    def gait_init_4(self):
        logging.info("Initialising Gait 4")
        self.gait_change_requested = False  # Reset the request flag
        # Update the min and max RPM for this gait:
        self.MIN_RPM = self.gait4_params['min_rpm']
        self.MAX_RPM = self.gait4_params['max_rpm']
        self.SMOOTHNESS = self.gait4_params['smoothness']
        self.odd_even = 0
        self.wheg_rpm = 0        

        self.positions = { 1: self.gait4_params['low_pos'], 2: self.gait4_params['high_pos'], 3: self.gait4_params['low_pos'], 4: self.gait4_params['high_pos'], 5: self.gait4_params['low_pos'], 6: self.gait4_params['high_pos'] }
        wait_time = 0.5
        logging.info(f"Initialised Gait 4, waiting for {wait_time} seconds")
        return
    
    
    def execute_gait(self, throttle):
        """Execute the current gait calculations."""
        if not self.SHUT_DOWN:
            
            if self.turn_mode_active:
                logging.debug("Turn mode activated executing turn gait")
                turn_function = self.turn_mode()
                if self.turn_mode_deactivate:
                    self.turn_mode_off()
                    logging.info("Turn mode deactivating")

            # Get the current gait function and execute it
            self.velocity = throttle
            logging.info(f"Throttle recieved for gait: {self.velocity}")
            gait_function = self.gait_methods[self.current_gait_index]
            
            # Handle turn mode
            if self.turn_mode_requested:
                self.init_turn_mode()
                logging.info("Turn mode activated.")

        else:
            logging.debug("Emergency stop activated, gait execution paused.")
    
    def gait_1(self):
        """Execute Gait 1 and return how long to wait before the next step."""
        logging.debug("Executing Gait 1")
        logging.info(f"Current velocity input for Gait 1: {self.velocity}")
        self.wheg_rpm = self.adjust_wheg_rpm(self.velocity)
        if self.wheg_rpm > 1 and self.gait_change_requested == False:
        
            self.increment = 360  # Example movement angle
            
            self.velocities = {1: self.wheg_rpm, 2: self.wheg_rpm, 3: self.wheg_rpm, 4: self.wheg_rpm, 5: self.wheg_rpm, 6: self.wheg_rpm}
            self.increments = {1: self.increment, 2: self.increment, 3: self.increment, 4: self.increment, 5: self.increment, 6: self.increment}

            # Calculate wait time based on RPM (example formula: degrees moved / (6 * RPM))self
            wait_time = self.increment / (6 * self.wheg_rpm)
            logging.info(f"Gait 1 step executed at {self.wheg_rpm:.2f}RPM, wait for {wait_time:.2f} seconds")
            return wait_time
        return 0  # No movement, no wait time

    def gait_2(self):
        """Execute Gait 2 and return how long to wait before the next step."""
        logging.debug("Executing Gait 2")
        self.wheg_rpm = self.adjust_wheg_rpm(self.velocity)
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

            # Set profile velocities and self.self.increments
            self.velocities = {1: rpm_1, 2: rpm_2, 3: rpm_1, 4: rpm_2, 5: rpm_1, 6: rpm_2}
            self.increments = {1: inc_1, 2: inc_2, 3: inc_1, 4: inc_2, 5: inc_1, 6: inc_2}

            # Calculate wait time
            wait_time = (inc_1 / (6 * rpm_1))+self.gait2_params['delay']
            self.odd_even += 1
            logging.info(f"Gait 2 step executed at {self.wheg_rpm:.2f}RPM, wait for {wait_time:.2f} seconds")
            return wait_time
        return 0  # No movement, no wait time

    def gait_3(self):
        """Execute Gait 3 and return how long to wait before the next step."""
        logging.debug("Executing Gait 3")
        self.wheg_rpm = self.adjust_wheg_rpm(self.velocity)
        
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

            # Set profile velocities for all whegs
            self.velocities = {1: rpm_1, 2: rpm_2, 3: rpm_3, 4: rpm_1, 5: rpm_2, 6: rpm_3}
            self.increments = {1: inc_1, 2: inc_2, 3: inc_3, 4: inc_1, 5: inc_2, 6: inc_3}

            # Calculate wait time based on the largest movement (300 degrees)
            wait_time = (self.gait3_params['fast_ang'] / (6 * self.wheg_rpm)) + self.gait3_params['delay']
            self.odd_even += 1
            logging.info(f"Gait 3 step executed at {self.wheg_rpm:.2f} RPM, wait for {wait_time:.2f} seconds")
            return wait_time

        return 0  # No movement, no wait time

    def gait_4(self):
        """Execute Gait 4 and return how long to wait before the next step."""
        logging.debug("Executing Gait 4")
        self.wheg_rpm = self.adjust_wheg_rpm(self.velocity)
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

            # Set profile velocities and self.self.increments
            self.velocities = {1: rpm_1, 2: rpm_2, 3: rpm_1, 4: rpm_2, 5: rpm_1, 6: rpm_2}
            self.increments = {1: inc_1, 2: inc_2, 3: inc_1, 4: inc_2, 5: inc_1, 6: inc_2}

            # Calculate wait time
            wait_time = (inc_1 / (6 * rpm_1))+self.gait2_params['delay']
            self.odd_even += 1
            logging.info(f"Gait 4 step executed at {self.wheg_rpm:.2f}RPM, wait for {wait_time:.2f} seconds")
            return wait_time
        return 0  # No movement, no wait time
    
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
        if trigger_value == -1.0: # Low trigger value, ensure velocity is 0
            self.wheg_rpm = 0
        logging.debug(f"Adjusted wheg speed: target_rpm={target_rpm}, current_rpm={self.wheg_rpm}")
        return self.wheg_rpm
    
    def get_velocities(self):
        """Return the current velocities of the whegs."""
        return self.velocities
    
    def get_increments(self):
        """Return the current increments of the whegs."""
        return self.increments
    
    def get_positions(self):
        """Return the current positions of the whegs."""
        return self.positions
    
    def get_shutoff_positions(self):
        """Return the positions to shut off the whegs."""
        self.positions = { 1: self.gait4_params['high_pos'], 2: self.gait4_params['high_pos'], 3: self.gait4_params['high_pos'], 4: self.gait4_params['high_pos'], 5: self.gait4_params['high_pos'], 6: self.gait4_params['high_pos'] }
        return self.positions
