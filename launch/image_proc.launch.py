
from launch import LaunchDescription
from launch_ros import actions

from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ld = LaunchDescription()

    # Load composable container
    image_processing = actions.ComposableNodeContainer(
        name='image_proc_container',
        package='rclcpp_components',
        executable='component_container',
        namespace='',
        composable_node_descriptions=[
            ComposableNode(
                package='image_proc',
                plugin='image_proc::DebayerNode',
                name='debayer_node',
            ),
            # Example of rectifying an image
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_mono_node',
                # Remap subscribers and publishers
                remappings=[
                    # Subscriber remap
                    ('image', 'image_mono'),
                    ('camera_info', 'camera_info'),
                    ('image_rect', 'image_rect')
                ],
            ),
            # Example of rectifying an image
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_color_node',
                # Remap subscribers and publishers
                remappings=[
                    # Subscriber remap
                    ('image', 'image_color'),
                    # Publisher remap
                    ('image_rect', 'image_rect_color')
                ],
            )],
        output='screen'
    )

    ld.add_action(image_processing)

    return ld
