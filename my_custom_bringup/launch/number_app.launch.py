from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    remap_number_topic = ("number", "my_number")

    number_publisher_node = Node(
        package="cpp_pkg_ws",
        executable="number_publisher",
        name="my_number_publisher",
        remappings=[
            remap_number_topic
        ],
        parameters=[
            {"number_to_publish": 4},
            {"publish_frequency": 5.0}
        ]
    )
    
    counter_reset_node = Node(
        package="cpp_pkg_ws",
        executable="counter_reset",
        name="my_reset_counter",
    )    

    number_counter_node = Node(
        package="py_pkg_ws",
        executable="number_counter",
        name="my_number_counter",
        remappings=[
            remap_number_topic,
            ("number_count", "my_number_count")
        ]
    )

    ld.add_action(number_publisher_node)
    ld.add_action(counter_reset_node)
    ld.add_action(number_counter_node)
    return ld