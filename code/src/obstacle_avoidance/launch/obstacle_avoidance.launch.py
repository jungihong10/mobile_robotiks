from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="obstacle_avoidance",
                executable="obstacle_avoidance",
                name="obstacle_avoidance",
                parameters=[
                    {"kAtt": 0.5},
                    {"kRep": 0.4},
                    {"d_thres": 0.4},
                    {"length": 0.5},
                    {"width": 0.5},
                ],
            )
        ]
    )
