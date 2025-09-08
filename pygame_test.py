import pygame

pygame.init()
pygame.joystick.init()

count = pygame.joystick.get_count()
print("Number of joysticks:", count)

for i in range(count):
    js = pygame.joystick.Joystick(i)
    js.init()
    print(f"Joystick {i}: {js.get_name()}")