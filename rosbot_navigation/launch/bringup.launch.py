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

import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
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
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Directories
    rosbot_navigation = FindPackageShare("rosbot_navigation")
    bringup_dir = FindPackageShare("nav2_bringup")
    launch_dir = PathJoinSubstitution([bringup_dir, "launch"])

    # Launch configuration variables
    common_params_file = LaunchConfiguration("common_params_file")
    controller = LaunchConfiguration("controller")
    log_level = LaunchConfiguration("log_level")
    map_path = LaunchConfiguration("map")
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    robot_model = LaunchConfiguration("robot_model")
    slam = LaunchConfiguration("slam")
    use_sim_time = LaunchConfiguration("use_sim_time")

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

    # Box around the robot body whose laser returns (self-reflections) are removed.
    # Larger than the footprint to also cover antenna/sensors and lidar uncertainty.
    laser_filter_box = {
        "rosbot": {"min_x": -0.17, "max_x": 0.10, "min_y": -0.12, "max_y": 0.12, "max_z": 0.2},
        "rosbot_xl": {"min_x": -0.245, "max_x": 0.165, "min_y": -0.145, "max_y": 0.145, "max_z": 0.3},
    }

    def prepare_params_files(context):
        ns = namespace.perform(context)
        model = robot_model.perform(context)
        share = rosbot_navigation.perform(context)

        def substitute(text):
            text = text.replace("<namespace>/", (ns + "/") if ns else "")
            if model in robot_footprint:
                fp = robot_footprint[model]
                for key in ("min_x", "max_x", "min_y", "max_y"):
                    text = text.replace(f"<{key}>", str(fp[key]))
            return text

        # Common base + controller file merged into one file. They are disjoint at the
        # top level (common = everything but controller_server; controller file =
        # controller_server only), so a shallow merge suffices. One merged file is
        # required: two separate ParameterFiles would drop nested/list params.
        common = yaml.safe_load(substitute(open(common_params_file.perform(context)).read()))
        controller = yaml.safe_load(substitute(open(params_file.perform(context)).read()))
        merged = {**common, **controller}

        laser = open(os.path.join(share, "config", "laser_filter.yaml")).read()
        if model in laser_filter_box:
            for key, value in laser_filter_box[model].items():
                laser = laser.replace(f"<lf_{key}>", str(value))

        fd, merged_path = tempfile.mkstemp(prefix="nav2_merged_", suffix=".yaml")
        with os.fdopen(fd, "w") as f:
            yaml.safe_dump(merged, f)
        fd, laser_path = tempfile.mkstemp(prefix="laser_filter_", suffix=".yaml")
        with os.fdopen(fd, "w") as f:
            f.write(laser)

        return [
            SetLaunchConfiguration("params_file", merged_path),
            SetLaunchConfiguration("laser_filter_params_file", laser_path),
        ]

    prepare_params_action = OpaqueFunction(function=prepare_params_files)

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    configured_laser_filter_params = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration("laser_filter_params_file"),
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

    params_filename = PythonExpression(["'nav2_' + '", controller, "' + '.yaml'"])
    declare_params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([rosbot_navigation, "config", params_filename]),
        description="Path to the controller-specific nav2 parameters file",
    )

    declare_common_params_file_arg = DeclareLaunchArgument(
        "common_params_file",
        default_value=PathJoinSubstitution(
            [rosbot_navigation, "config", "nav2_common.yaml"]
        ),
        description="Path to the common nav2 parameters file (shared across controllers)",
    )

    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable("ROBOT_MODEL", default_value=""),
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
                name="laser_filter",
                namespace="",
                package="laser_filters",
                executable="scan_to_scan_filter_chain",
                parameters=[configured_laser_filter_params],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
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
        declare_common_params_file_arg,
        declare_slam_arg,
        declare_use_sim_time_arg,
        prepare_params_action,
        PushROSNamespace(namespace),
        SetParameter(name="use_sim_time", value=use_sim_time),
        SetRemap("/diagnostics", "diagnostics"),
        SetRemap("/tf", "tf"),
        SetRemap("/tf_static", "tf_static"),
        bringup_group,
    ]

    return LaunchDescription(actions)
