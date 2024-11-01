import pygame
import time

# Initialize pygame and the joystick
pygame.init()
pygame.joystick.init()

# Ensure at least one joystick is connected
if pygame.joystick.get_count() == 0:
    print("No joystick connected.")
    pygame.quit()
else:
    # Initialize the first joystick (PlayStation 5 controller)
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Connected to joystick: {joystick.get_name()}")

    try:
        while True:
            # Pump pygame events to update joystick state
            pygame.event.pump()

            # Get the X and Y axes of the left stick
            left_stick_x = joystick.get_axis(0)  # Left stick horizontal
            left_stick_y = joystick.get_axis(1)  # Left stick vertical

            # Get the X and Y axes of the right stick
            right_stick_x = joystick.get_axis(3)  # Right stick horizontal
            right_stick_y = joystick.get_axis(4)  # Right stick vertical

            # Print the directions
            print(f"Left Stick:  X: {left_stick_x:.2f}, Y: {left_stick_y:.2f}")
            print(f"Right Stick: X: {right_stick_x:.2f}, Y: {right_stick_y:.2f}")

            # Sleep for a bit to reduce CPU usage
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Exiting program.")

    # Quit pygame when done
    pygame.quit()
