from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("swarm_random_payload")
    default_case_file = PathJoinSubstitution([package_share, "data", "demo8_case.json"])
    default_rviz_config = PathJoinSubstitution([package_share, "config", "cinematic_payload_demo.rviz"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("case_file", default_value=default_case_file),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("dt", default_value="0.03"),
            DeclareLaunchArgument("frames_per_tick", default_value="3"),
            DeclareLaunchArgument("initial_hold_seconds", default_value="1.0"),
            DeclareLaunchArgument("final_hold_seconds", default_value="1.0"),
            DeclareLaunchArgument("map_scale", default_value="0.01"),
            DeclareLaunchArgument("display_scale", default_value="0.018"),
            DeclareLaunchArgument("drone_mesh_scale", default_value="0.44"),
            DeclareLaunchArgument("follow_position_alpha", default_value="0.16"),
            DeclareLaunchArgument("follow_yaw_alpha", default_value="0.10"),
            DeclareLaunchArgument("follow_lock_yaw", default_value="false"),
            DeclareLaunchArgument("show_cinematic_hud", default_value="false"),
            DeclareLaunchArgument("control_mode", default_value=""),
            Node(
                package="swarm_random_payload",
                executable="random_payload_demo",
                name="cinematic_payload_demo",
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
                        "show_region_labels": False,
                        "show_bridge_labels": False,
                        "show_formation_layer_graph": False,
                        "show_formation_graph_edges": False,
                        "show_symbolic_route": False,
                        "show_agent_trajectories": False,
                        "show_current_status_text": False,
                        "show_cinematic_hud": ParameterValue(
                            LaunchConfiguration("show_cinematic_hud"), value_type=bool
                        ),
                        "show_controller_arrows": False,
                        "show_agent_labels": False,
                        "publish_centroid_path": False,
                        "publish_follow_tf": True,
                        "follow_frame": "payload_follow",
                        "follow_frame_height": 0.85,
                        "follow_lock_yaw": ParameterValue(LaunchConfiguration("follow_lock_yaw"), value_type=bool),
                        "follow_position_alpha": ParameterValue(
                            LaunchConfiguration("follow_position_alpha"), value_type=float
                        ),
                        "follow_yaw_alpha": ParameterValue(LaunchConfiguration("follow_yaw_alpha"), value_type=float),
                        "use_drone_mesh": True,
                        "show_propeller_blur": True,
                        "drone_mesh_scale": ParameterValue(LaunchConfiguration("drone_mesh_scale"), value_type=float),
                        "map_scale": ParameterValue(LaunchConfiguration("map_scale"), value_type=float),
                        "display_scale": ParameterValue(LaunchConfiguration("display_scale"), value_type=float),
                        "case_file": LaunchConfiguration("case_file"),
                        "control_mode": LaunchConfiguration("control_mode"),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="payload_cinematic_rviz",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                condition=IfCondition(LaunchConfiguration("start_rviz")),
            ),
        ]
    )
