import pygame

# Initialize pygame and joystick support
pygame.init()
pygame.joystick.init()

# Check for controllers
joystick_count = pygame.joystick.get_count()
print(f"Number of joysticks detected: {joystick_count}")

if joystick_count == 0:
    print("No controller found. Make sure your PS4 controller is paired via Bluetooth or USB.")
else:
    # Use the first joystick
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Connected to: {js.get_name()}")

    # Main loop
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # Button press
            if event.type == pygame.JOYBUTTONDOWN:
                print(f"Button {event.button} pressed")

            # Button release
            elif event.type == pygame.JOYBUTTONUP:
                print(f"Button {event.button} released")

            # Joystick movement (axes)
            elif event.type == pygame.JOYAXISMOTION:
                print(f"Axis {event.axis} moved to {event.value:.2f}")

            # D-pad (hat) movement
            elif event.type == pygame.JOYHATMOTION:
                print(f"D-pad moved: {event.value}")

pygame.quit()