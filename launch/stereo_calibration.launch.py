from launch import LaunchDescription
from launch_ros.actions import Node

# see https://github.com/ros-perception/image_pipeline/pull/597

# do it manually: 
#     ros2 run camera_calibration cameracalibrator --size=10x7 --square=0.026 --approximate=0.02 --no-service-check


def generate_launch_description():
    ld = LaunchDescription()
    
    ld.add_action(Node(
        package='camera_calibration', 
        executable='cameracalibrator', 
        output='screen',
        arguments=["--size","8x6", 
                   "--square","0.063", 
                   "--approximate","0.3", 
                   "--no-service-check"]
        ))

    return ld
