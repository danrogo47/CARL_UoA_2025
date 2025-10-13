"""
CARL client script.
Before running, change line 19 to the Rapsberry Pi IP address, and connect PS4 controller
via USB or Bluetooth.
"""

import pygame
import socket
import json
import time

# Initialise pygame
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Set up the socket connection
HOST = '10.13.81.202' # Daniel
# HOST = '172.20.10.2' # Jacob
PORT = 8001
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("Attempting to connect to Raspberry Pi...")
sock.connect((HOST, PORT))
print(f"Connected to Raspberry Pi at {HOST}:{PORT}")

def send_command(data):
    """Gets commands and sends them over the socket connection."""
    message = json.dumps(data)
    sock.sendall((message + "\n").encode('utf-8'))

try:
    print("Sending controller state to Raspberry Pi...")
    while True:
        # get controller inputs
        pygame.event.pump()
        axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
        buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]

        # Construct a dictionary of the controller state
        controller_state = {
            'axes': axes,
            'buttons': buttons
        }
        # Send the controller state to the Raspberry Pi
        send_command(controller_state)
        
        time.sleep(0.05)  # Adjust the sleep time as needed

except KeyboardInterrupt:
    print("Exiting...")

finally:
    sock.close()
    pygame.quit()
    print("Connection closed and pygame quit.")