from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Static image publisher
    static_image_publisher_node = Node(
        package='computer_vision',
        executable='static_image_publisher_node',
        name='static_image_publisher_node'
    )

    # Image blur node
    image_blur_node = Node(
        package='computer_vision',
        executable='image_blur_node',
        name='image_blur_node'
    )

    # Image edge detection node with 'canny' method (default)
    image_edge_node = Node(
        package='computer_vision',
        executable='image_edge_detection_node',
        name='image_edge_detection_node',
        arguments=['canny']
    )

    return LaunchDescription([
        static_image_publisher_node,
        image_blur_node,
        image_edge_node
    ])
