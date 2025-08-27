import rclpy
from rclpy.node import Node
from time import sleep
import logging
from std_msgs.msg import Bool
from std_msgs.msg import String
from custom_msgs.msg import Joint
from carl_controller.wheg_plugin.dynamixel_control import DynamixelController

class JointNode(Node):
        
    def __init__(self):
    
        super().__init__('joint') # Remove if not necessary
        
        self.dynamixel = DynamixelController()
    
        # Set position limits for the pivot motors
        self.dynamixel.set_drive_mode_group('Pivot_Group', False)
        self.dynamixel.set_position_limits_group('Pivot_Group', self.config['position_limits']['Hinges']['min_degrees'], self.config['position_limits']['Hinges']['max_degrees'])
        logging.info("Set position limits for the pivot motors")
        
        # From RE-RASSOR - Remove if not necessary
        self.subscription_1 = self.create_subscription(Bool, 'shutdown_cmd', self.shutdown_callback, 10)
        self.subscription_2 = self.create_subscription(Joint, 'joint_cmd', self.listener_callback, 10)
        
        
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


    async def control_pivots_with_dpad(self): # May need to add 'dpad_inputs' as a variable to be listening for similar to how RE-RASSOR has done it for the T-joint movements in 'def listener_callback(self, msg):'
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

    def listener_callback(self, msg):
        if self.SHUT_DOWN:
            return
        # shutdown flag true: immediately stop motor movement
        # if self.SHUT_DOWN:
        #     self.kit.pivot1.release()
        #     self.kit.pivot2.release()
        #     return
        
        # # set movement direction based on controller input
        # if msg.up == 1:
        #     self.direction = pivot.UP
        # elif msg.down == 1:
        #     self.direction = pivot.DOWN

        # # move
        # if msg.up == 1 or msg.down == 1:
        #     if msg.joint.data == 'FRONT':
        #         for _ in range(self.steps):
        #             self.kit.pivot1.onestep(direction=self.direction)
        #             sleep(0.01)  # 10 milliseconds delay

        #     elif msg.joint.data == 'BACK':
        #         for _ in range(self.steps):
        #             self.kit.pivot2.onestep(direction=self.direction)
        #             sleep(0.01)  # 10 milliseconds delay
                    
        # # if no longer moving, release
        # else:
        #     self.kit.pivot1.release()
        #     self.kit.pivot2.release()

def main(args=None):
    rclpy.init(args=args)

    node = JointNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
