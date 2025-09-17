# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushROSNamespace, SetParameter, SetRemap
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    # Directories
    rosbot_navigation = FindPackageShare("rosbot_navigation")
    bringup_dir = FindPackageShare("nav2_bringup")
    launch_dir = PathJoinSubstitution([bringup_dir, "launch"])

    # Launch configuration variables
    controller = LaunchConfiguration("controller")
    log_level = LaunchConfiguration("log_level")
    map_path = LaunchConfiguration("map")
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    robot_model = LaunchConfiguration("robot_model")
    slam = LaunchConfiguration("slam")
    use_sim_time = LaunchConfiguration("use_sim_time")

    namespace_ext = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])

    robot_footprint = {
        "rosbot": {
            "min_x": -0.10,
            "min_y": -0.12,
            "max_x": 0.10,
            "max_y": 0.12,
        },
        "rosbot_xl": {
            "min_x": -0.17,
            "min_y": -0.16,
            "max_x": 0.17,
            "max_y": 0.16,
        },
    }

    def override_params_file(robot_model_name):
        footprint = robot_footprint[robot_model_name]
        params = ReplaceString(
            source_file=params_file,
            replacements={
                "<namespace>/": namespace_ext,
                "<min_x>": str(footprint["min_x"]),
                "<max_x>": str(footprint["max_x"]),
                "<min_y>": str(footprint["min_y"]),
                "<max_y>": str(footprint["max_y"]),
            },
            condition=IfCondition(
                PythonExpression(["'", robot_model, f"' == '{robot_model_name}'"])
            ),
        )

        return params

    params_file = override_params_file("rosbot")
    params_file = override_params_file("rosbot_xl")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    declare_controller_arg = DeclareLaunchArgument(
        "controller",
        default_value="mppi",
        description="Nav2 controller type",
        choices=["dwb", "mppi", "rpp"],
    )

    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level",
        choices=["debug", "info", "warning", "error"],
    )

    declare_map_arg = DeclareLaunchArgument(
        "map", default_value="", description="Full path to map yaml file to load"
    )

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes",
    )

    params_filename = PythonExpression(["'nav2_' + '", controller, "' + '_params.yaml'"])
    declare_params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([rosbot_navigation, "config", params_filename]),
        description="Path to the nav2 parameters file",
    )

    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable("ROBOT_MODEL_NAME", default_value=""),
        description="Specify robot model",
        choices=["rosbot", "rosbot_xl"],
    )

    declare_slam_arg = DeclareLaunchArgument(
        "slam", default_value="True", description="Whether run a SLAM"
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    # Specify the actions
    bringup_group = GroupAction(
        [
            Node(
                name="nav2_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[configured_params, {"autostart": "True"}],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([rosbot_navigation, "launch", "slam.launch.py"])
                ),
                condition=IfCondition(slam),
                launch_arguments={
                    "namespace": namespace,
                    "params_file": params_file,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([launch_dir, "localization_launch.py"])
                ),
                condition=UnlessCondition(slam),
                launch_arguments={
                    "container_name": "nav2_container",
                    "map": map_path,
                    "namespace": namespace,
                    "params_file": params_file,
                    "use_composition": "True",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([rosbot_navigation, "launch", "navigation.launch.py"])
                ),
                launch_arguments={
                    "container_name": "nav2_container",
                    "namespace": namespace,
                    "params_file": params_file,
                    "use_composition": "True",
                }.items(),
            ),
            Node(
                condition=IfCondition(slam),
                name="map_autosaver",
                package="rosbot_navigation",
                executable="map_autosaver_node",
                parameters=[{"autosave_period": 30.0}],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
        ]
    )

    actions = [
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        declare_controller_arg,
        declare_log_level_arg,
        declare_map_arg,
        declare_namespace_arg,
        declare_robot_model_arg,
        declare_params_file_arg,
        declare_slam_arg,
        declare_use_sim_time_arg,
        PushROSNamespace(namespace),
        SetParameter(name="use_sim_time", value=use_sim_time),
        SetRemap("/diagnostics", "diagnostics"),
        SetRemap("/tf", "tf"),
        SetRemap("/tf_static", "tf_static"),
        bringup_group,
    ]

    return LaunchDescription(actions)
