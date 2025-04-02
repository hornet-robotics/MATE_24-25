import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Launch nodes from Package 1
        launch_ros.actions.Node(
            package='joystick_module',
            executable='joystick_publisher',
            name='joystick_publisher'
        ),
        launch_ros.actions.Node(
            package='vector_drive',
            executable='vector_drive_subscriber',
            name='vector_drive_subscriber'
        ),
    ])
