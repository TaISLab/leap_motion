# Based on Lucas Walter's launch file in usb_cam


import argparse
from launch import LaunchDescription
from launch_ros.actions import Node

import os
import sys

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    leap_motion_dir = get_package_share_directory('leap_motion')

    # get path to params files
    params_path_left = os.path.join(
        leap_motion_dir,
        'config',
        'params_left.yaml'
    )
    params_path_right = os.path.join(
        leap_motion_dir,
        'config',
        'params_right.yaml'
    )
    
    ld.add_action(Node(
        package='usb_cam', executable='usb_cam_node_exe', output='screen',
        name="dcha",
        namespace="right",
        parameters=[params_path_right]
        ))

    ld.add_action(Node(
        package='usb_cam', executable='usb_cam_node_exe', output='screen',
        name="izda",
        namespace="left",
        parameters=[params_path_left]
        ))


    return ld
