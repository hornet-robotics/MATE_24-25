# __init__.py
import pygame

def init_joysticks():
    pygame.init()
    pygame.joystick.init()
    joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
    for joystick in joysticks:
        joystick.init()
    return joysticks

# pygame.init()
#     pygame.joystick.init()
#     joysticks=[]
#     for event in pygame.event.get():
#         if event.type == pygame.JOYDEVICEADDED:
#             joy = pygame.joystick.Joystick(event.device_index)
#             joysticks.append(joy)
#     return joysticks