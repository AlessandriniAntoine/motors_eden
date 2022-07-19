from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():



################### Motors ##############

    motors = Node(
        package="motors_eden",
        executable="motors",
        name = "MotorsRosNode",
        namespace= "Motors",
        output='screen',
        remappings=[('/Motors/ref/displacement','/Command')],
        parameters=[
        {'ids': [1,2,3,4],
        'gearRatio': [80/20,110/25,105/21,50/25],
        }
    ],
    )

    ############## create launch object ##############
    launch_description = LaunchDescription()

    launch_description.add_action(motors)

    return launch_description