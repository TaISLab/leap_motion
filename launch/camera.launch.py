from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python import get_package_share_directory

def generate_launch_description():

    # Default filenames and where to find them
    leap_motion_dir = get_package_share_directory('leap_motion')
    print('..................................')
    print(leap_motion_dir)
    print('..................................')
    
    default_params_file = os.path.join(leap_motion_dir, 'configuration', 'nav2_params.yaml')

    ld = LaunchDescription()

    # Set env var to print messages to stdout immediately
    log_config = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    ld.add_action(log_config)

    # Create the launch configuration variables: 
    log_level = LaunchConfiguration('log_level')

    # Map these variables to Arguments: can be set from the command line or a default will be used
    log_level_launch_arg = DeclareLaunchArgument("log_level", default_value=TextSubstitution(text="DEBUG"))

    # Declare defined launch options 
    ld.add_action(log_level_launch_arg)

    # gets raw images from the Leap Motion controller
    camera_node = Node(package = "leap_motion", 
                executable = "leap_motion_camera_node",
                name="leap_camera", 
                output="screen"
          )
    ld.add_action(camera_node)

    #  generates the pointcloud 
    # ros2 launch stereo_image_proc stereo_image_proc.launch.py
    stereo_image_include = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(leap_motion_dir, 'launch/stereo_image_proc.launch.py')),
            launch_arguments={
                'log_level': log_level          
            }.items()
        )
    ])
    ld.add_action(stereo_image_include)

    # TODO: this is orientative ...
    transform_node = Node(package = "tf2_ros", 
                executable = "static_transform_publisher",
                name="link2_broadcaster", 
                arguments = ['0.03', '0', '0', '0', '0', '0', 'leap_camera_left', 'leap_camera_right']
          )
    ld.add_action(transform_node)

    return ld
