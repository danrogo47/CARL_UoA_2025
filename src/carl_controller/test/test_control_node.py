import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

import re_rassor_controller.lib.controller_input_defs as inputs

class TestControlNode(Node):
    def __init__(self):
        super().__init__('test_control_node')

        self.v_front_left = 0
        self.v_back_left = 0
        self.v_front_right = 0
        self.v_back_right = 0

        self.last_called_time = time.time()

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)

    def listener_callback(self, msg):

        current_time = time.time()

        # only send cmds every 0.5 s
        if (current_time - self.last_called_time) > 0.5:

            velocities = self.calculate_motor_velocities(msg)
            self.last_called_time = current_time

    def calculate_motor_velocities(self, msg):

        x_cmd = msg.linear.x
        z_cmd = msg.angular.z
        
        # constants
        speed_multiplier = 100
        turn_threshold = 0.2
        turn_component = 0

        # set turn component based on a threshold value
        if abs(z_cmd) > turn_threshold:

            if z_cmd > 0:
                turn_component = z_cmd - turn_threshold
            
            else:
                turn_component = z_cmd + turn_threshold

        # left wheels
        raw_v_front_left = (-0.5*x_cmd + 0.5*turn_component)*speed_multiplier
        raw_v_back_left = (-0.5*x_cmd + 0.5*turn_component)*speed_multiplier

        # right wheels
        raw_v_front_right = (-0.5*x_cmd - 0.5*turn_component)*speed_multiplier
        raw_v_back_right = (-0.5*x_cmd - 0.5*turn_component)*speed_multiplier

        # ease the speeds
        self.v_front_left = self.ease_speed(raw_v_front_left, self.v_front_left)
        self.v_back_left = self.ease_speed(raw_v_back_left, self.v_back_left)
        self.v_front_right = self.ease_speed(raw_v_front_right, self.v_front_right)
        self.v_back_right = self.ease_speed(raw_v_back_right, self.v_back_right)

        return self.v_front_left, self.v_back_left, self.v_front_right, self.v_back_right
    
    def ease_speed(self, new_speed, prev_speed):

        max_delta = 10

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

        # update the speed and return
        return corrected_speed

def main(args=None):

    rclpy.init(args=args)
    node = TestControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
