import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
from ament_index_python.packages import get_package_share_directory


def noisy_controller(context, *args, **kwargs):
    use_python = LaunchConfiguration("use_python")
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))

    noisy_controller_py = Node(
        package="bumperbot_controller",
        executable="noisy_controller.py",
        parameters=[
            {"wheel_radius": wheel_radius + wheel_radius_error,
             "wheel_separation": wheel_separation + wheel_separation_error}],
        condition=IfCondition(use_python),
    )

    noisy_controller_cpp = Node(
        package="bumperbot_controller",
        executable="noisy_controller",
        parameters=[
            {"wheel_radius": wheel_radius + wheel_radius_error,
             "wheel_separation": wheel_separation + wheel_separation_error}],
        condition=UnlessCondition(use_python),
    )

    return [
        noisy_controller_py,
        noisy_controller_cpp,
    ]



def generate_launch_description():
    
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True",
    )
    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="True",
    )
    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False",
    )
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033",
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17",
    )
    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005",
    )
    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value="0.02",
    )
    
    is_sim = LaunchConfiguration("is_sim")
    use_simple_controller = LaunchConfiguration("use_simple_controller")
    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("bumperbot_description"),
                    "urdf",
                    "bumperbot.urdf.xacro",
                ),
                " is_sim:=False"
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        condition=UnlessCondition(is_sim),
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": is_sim},
            os.path.join(
                get_package_share_directory("bumperbot_controller"),
                "config",
                "bumperbot_controllers.yaml",
            ),
        ],
        condition=UnlessCondition(is_sim),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bumperbot_controller", 
                   "--controller-manager", 
                   "/controller_manager"
        ],
        condition=UnlessCondition(use_simple_controller),
    )

    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["simple_velocity_controller", 
                        "--controller-manager", 
                        "/controller_manager"
                ]
            ),
            Node(
                package="bumperbot_controller",
                executable="simple_controller.py",
                parameters=[
                    {"wheel_radius": wheel_radius,
                    "wheel_separation": wheel_separation}],
                condition=IfCondition(use_python),
            ),
            Node(
                package="bumperbot_controller",
                executable="simple_controller",
                parameters=[
                    {"wheel_radius": wheel_radius,
                    "wheel_separation": wheel_separation}],
                condition=UnlessCondition(use_python),
            ),
        ]
    )

    noisy_controller_launch = OpaqueFunction(function=noisy_controller)

    return LaunchDescription(
        [
            is_sim_arg,
            use_simple_controller_arg,
            use_python_arg,
            wheel_radius_arg,
            wheel_separation_arg,
            wheel_radius_error_arg,
            wheel_separation_error_arg,
            robot_state_publisher_node,
            controller_manager,
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
            simple_controller,
            noisy_controller_launch,
        ]
    )