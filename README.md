# Mate Robotics Software 

## Mate Architecture
<div align="center">
    <img src="/media/mate_architecture.jpg" alt="mate_architecture" width="70%">
</div>

## Joystick Module

The joystick module is used to get input of joysticks and buttons of a connected Playstation or Xbox controller. This module uses the pygame library. This information is recorded and stored in an array for manipulation.

### Initial Setup

``` python
def __init__(self):
        super().__init__('publisher_node') 
        self.publisher_ = self.create_publisher(Float32MultiArray, 'joystick_topic', 10) 
        timer_period = 0.0000000001  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        pygame.init() 
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
```
This first part of the code initizalizes the node for ROS to detect and run. A timer is set to not overload the receiver, currently set to 1 ns. Lastly, pygame is initialized and the controller is set.

### Button Information

```python
    def timer_callback(self):
        pygame.event.pump()
        result= [] 
        left_stick_x = self.joystick.get_axis(0)  
        result.append(left_stick_x)
        left_stick_y = self.joystick.get_axis(1)  
        result.append(left_stick_y)

        right_stick_x = self.joystick.get_axis(3)  
        result.append(right_stick_x)
        right_stick_y = self.joystick.get_axis(4)  
        result.append(right_stick_y)

        msg = Float32MultiArray()
        msg.data = result 
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
```

This is part of the code where an event is checked on the controller, and those values are put into an array and transmitted to the receiver which will do actions such as adjusting the thrusters. Listed below is all the data that is tracked, what numerical value the calls return, and how it is arranged in the array. In addition, the guide or PS button can be used to end the program.

| Button/Joystick|Pygame Equivalent| Numerical Value | Array Postion in Message
| :-----------------------:|:--------------------|:------------------|:-----------|
| Left Stick X-Direction       | joystick.get_axis(0)|Left: -1, No Input: 0, Right: 1 | 0 |
| Left Stick Y-Direction     | joystick.get_axis(1) |Up: -1, No Input: 0, Down: 1| 1 |
| Right Stick X-Direction  | joystick.get_axis(3)| Left: -1, No Input: 0, Right: 1 | 2 |
| Right Stick Y-Direction  | joystick.get_axis(4) | Up: -1, No Input: 0, Down: 1| 3 |
| Left Trigger Press  | joystick.get_axis(2)  | No Input: -1, Completely Pressed: 1| 4 |
| Right Trigger Press  | joystick.get_axis(5)  | No Input: -1, Completely Pressed: 1| 5 |
| D-Pad X-Direction | joystick.get_hat(0)[0] | Left: -1, No Input: 0, Right: 1 | 6 |
| D-Pad Y-Drection | joystick.get_hat(0)[1] | Up: -1, No Input: 0, Down: 1| 7 |
| X On Playstation, A On Xbox | joystick.get_button(0) | No Input: 0, Pressed: 1| 8 |
| Circle on Playstation, B on Xbox | joystick.get_button(1) | No Input: 0, Pressed: 1| 9 | 
| Triangle on Playstation, Y on Xbox | joystick.get_button(2) |No Input: 0, Pressed: 1| 10 |
| Square on Playstation, X on Xbox | joystick.get_button(3) |No Input: 0, Pressed: 1| 11 |
| Left Bumper | joystick.get_button(4) |No Input: 0, Pressed: 1| 12 | 
| Right Bumper | joystick.get_button(5) |No Input: 0, Pressed: 1| 13 |
| PS Button on Playstation, Guide Button on Xbox | joystick.get_button(10) |No Input: 0, Pressed: 1| N/A | 
