from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String, Int16
from geometry_msgs.msg import Twist
from custom_msgs.msg import WhegFeedback
from CARL_UoA_2025.src.carl_controller.carl_controller.wheg_plugin.gait_controller import GaitController
import rclpy
import logging
import yaml
import time
import atexit

class WhegMotorDrive(Node):
    def __init__(self):

        super().__init__('wheg_drive')
        
        with open('config.yaml', 'r') as file:
            self.config = yaml.safe_load(file)

        # initialise the wheg controller functions
        self.gait = GaitController(self.config)
            
        self.gait.setup_variables()
        self.initialise_direction()
        self.execute_gait_change()

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
            return
        
        if self.gait_change_requested:
            self.execute_gait_change()
        
        current_time = time.time()

        # only send commands every set time interval dt
        if (current_time - self.last_called_time) > self.dt:

            self.calculate_gait_velocities(msg)
            self.last_called_time = current_time
            
            self.drive_motors()

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
            increments = self.gait.get_increments()

    def execute_gait_change(self):
        """
        Executes the gait change by calling the gait controller's method.
        This method is called when the gait change is requested.
        """
        self.gait.execute_gait_change()
        
        self.gait.get_positions()
        
        self.gait_change_requested = False
        
    def initialise_direction(self):
        """
        Initialises the direction of the wheg motors based on the configuration.
        This method sets the initial direction for the wheg motors.
        """
        
        try:
            direction = {1 : 0, 2 : 0, 3 : 0, 4 : 1, 5 : 1, 6 : 1}
        except Exception as e:
            logging.error(f"Failed to set direction: {e}")


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