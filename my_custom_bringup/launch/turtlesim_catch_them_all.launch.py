from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    turtle_spawner_node = Node(
        package="py_turtlesim_catch_them_all",
        executable="turtle_spawner",
        parameters=[
            {"spawn_frequency": 1.0},
            {"turtle_name_prefix": "my_turtle"}
        ]
    )

    turtle_controller_node = Node(
        package="py_turtlesim_catch_them_all",
        executable="turtlesim_controller",
        parameters=[
            {"catch_closest_turtle_first": True}
        ]
    )

    ld.add_action(turtlesim_node)
    ld.add_action(turtle_spawner_node)
    ld.add_action(turtle_controller_node)
    return ld