from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_case_file = PathJoinSubstitution(
        [FindPackageShare("swarm_random_payload"), "data", "demo1_case.json"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("case_file", default_value=default_case_file),
            DeclareLaunchArgument("dt", default_value="0.08"),
            DeclareLaunchArgument("frames_per_tick", default_value="1"),
            DeclareLaunchArgument("initial_hold_seconds", default_value="0.0"),
            DeclareLaunchArgument("final_hold_seconds", default_value="0.0"),
            DeclareLaunchArgument("map_scale", default_value="0.01"),
            DeclareLaunchArgument("display_scale", default_value="0.018"),
            DeclareLaunchArgument("control_mode", default_value=""),
            Node(
                package="swarm_random_payload",
                executable="random_payload_demo",
                name="case_payload_demo",
                output="screen",
                parameters=[
                    {
                        "dt": ParameterValue(LaunchConfiguration("dt"), value_type=float),
                        "frames_per_tick": ParameterValue(LaunchConfiguration("frames_per_tick"), value_type=int),
                        "initial_hold_seconds": ParameterValue(
                            LaunchConfiguration("initial_hold_seconds"), value_type=float
                        ),
                        "final_hold_seconds": ParameterValue(
                            LaunchConfiguration("final_hold_seconds"), value_type=float
                        ),
                        "loop_demo": True,
                        "publish_frame": "map",
                        "show_region_labels": True,
                        "show_bridge_labels": True,
                        "show_formation_layer_graph": False,
                        "show_formation_graph_edges": False,
                        "show_symbolic_route": False,
                        "show_current_status_text": False,
                        "publish_centroid_path": False,
                        "map_scale": ParameterValue(LaunchConfiguration("map_scale"), value_type=float),
                        "display_scale": ParameterValue(LaunchConfiguration("display_scale"), value_type=float),
                        "case_file": LaunchConfiguration("case_file"),
                        "control_mode": LaunchConfiguration("control_mode"),
                    }
                ],
            )
        ]
    )
