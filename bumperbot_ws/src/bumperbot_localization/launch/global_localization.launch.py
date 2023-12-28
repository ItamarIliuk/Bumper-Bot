import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory

def generate_launch_description():
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    
    declare_map_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_maps"),
            "maps",
            "small_house",
            "map.yaml"
        ),
        description="Full path to map yaml file to load"
    )
    
    declare_amcl_config_arg = DeclareLaunchArgument(
        "amcl_config",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "config",
            "amcl.yaml"
        ),
        description="Full path to amcl yaml file to load"
    )
    
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        emulate_tty=True,
        parameters=[{"yaml_filename": LaunchConfiguration("map"),
                     "use_sim_time" : LaunchConfiguration("use_sim_time")}],
    )
    
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("amcl_config")],
    )
    
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{"node_names": ["map_server", "amcl"],
                     "use_sim_time" : LaunchConfiguration("use_sim_time"),
                     "autostart": True}],
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_map_arg,
        declare_amcl_config_arg,
        map_server,
        amcl,
        lifecycle_manager,
    ])