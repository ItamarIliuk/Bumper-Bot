import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    map_name_arg = DeclareLaunchArgument(
        "map_name"
    )
    
    # Start nav2_map_server
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[configured_params],
    )
    
    # Start amcl
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[configured_params],
    )

    return LaunchDescription([
        map_server,
        amcl,  
    ])