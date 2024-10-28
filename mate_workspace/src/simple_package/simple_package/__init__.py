import pygame

pygame.joystick.init()

joysticks = []

for event in pygame.event.get():
  if event.type == pygame.JOYDEVICEADDED:
    joy = pygame.joystick.Joystick(event.device_index)
    joysticks.append(joy)