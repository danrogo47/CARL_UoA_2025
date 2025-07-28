import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import time

class ControllerStateMonitor(Node):
    def __init__(self):
        super().__init__('controller_state_monitor')
        
        # Subscribing to 'controller_state' topic
        self.subscription = self.create_subscription(
            String,
            'controller_state',
            self.controller_state_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.topic_active = False
        self.last_received_time = time.time()
        self.timeout_seconds = 5.0  # Adjust timeout as needed
        self.process = None
        
        # Create a timer to periodically check if topic is still publishing
        self.timer = self.create_timer(1.0, self.check_timeout)
    
    def controller_state_callback(self, msg):
        # Update the time when the last message was received
        self.last_received_time = time.time()

        if not self.topic_active:
            self.get_logger().info('Controller state detected, launching other nodes...')
            self.launch_other_nodes()
            self.topic_active = True

    def check_timeout(self):
        # Check if the topic hasn't published within the timeout period
        if self.topic_active and (time.time() - self.last_received_time > self.timeout_seconds):
            self.get_logger().info('Controller state stopped, terminating other nodes...')
            self.terminate_other_nodes()
            self.topic_active = False
    
    def launch_other_nodes(self):
        # Launch other nodes using a subprocess call
        self.process = subprocess.Popen(["ros2", "launch", "carl_controller", "motor_control_nodes_launch.py"])
        self.get_logger().info("launching other nodes")

    def terminate_other_nodes(self):
        # Terminate the launched nodes if they're running
        if self.process:
            self.process.terminate()
            self.process.wait()
            self.process = None
            self.get_logger().info('Other nodes terminated.')
        self.get_logger().info('Other nodes terminated.')

def main(args=None):
    rclpy.init(args=args)
    monitor_node = ControllerStateMonitor()
    
    rclpy.spin(monitor_node)
    
    monitor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
