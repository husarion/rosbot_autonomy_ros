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
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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
    config_dir = LaunchConfiguration("config_dir")
    controller = LaunchConfiguration("controller")
    log_level = LaunchConfiguration("log_level")
    map_path = LaunchConfiguration("map")
    map_save_path = LaunchConfiguration("map_save_path")
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    preflight = LaunchConfiguration("preflight")
    preflight_timeout = LaunchConfiguration("preflight_timeout")
    robot_model = LaunchConfiguration("robot_model")
    slam = LaunchConfiguration("slam")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Same convention as the rosbot_ros packages: config_dir points at a writable copy
    # of the shipped config trees (`ros2 run rosbot_utils create_config_dir <dst>`), and
    # each package reads <config_dir>/<pkg>/config/. Empty falls back to the package share.
    pkg_config_path = PythonExpression(
        [
            "'",
            config_dir,
            "/rosbot_navigation/config' if '",
            config_dir,
            "' else '",
            rosbot_navigation,
            "/config'",
        ]
    )

    robot_footprint = {
        "rosbot": {"min_x": -0.10, "min_y": -0.12, "max_x": 0.10, "max_y": 0.12},
        "rosbot_xl": {"min_x": -0.17, "min_y": -0.16, "max_x": 0.17, "max_y": 0.16},
    }

    # Box around the robot body whose laser returns (self-reflections) are removed.
    # Larger than the footprint to also cover antenna/sensors and lidar uncertainty.
    laser_filter_box = {
        "rosbot": {"min_x": -0.17, "min_y": -0.12, "max_x": 0.10, "max_y": 0.12, "max_z": 0.2},
        "rosbot_xl": {"min_x": -0.25, "min_y": -0.15, "max_x": 0.17, "max_y": 0.15, "max_z": 0.3},
    }

    def prepare_params_files(context):
        ns = namespace.perform(context)
        model = robot_model.perform(context)
        config_path = pkg_config_path.perform(context)
        save_path = map_save_path.perform(context)

        # map_saver writes through ImageMagick, which reports a bare "Unable to open
        # file" if the directory is missing — and map_autosaver retries forever.
        os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)

        def substitute(text):
            text = text.replace("<namespace>/", (ns + "/") if ns else "")
            text = text.replace("<map_save_path>", save_path)
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

        laser = open(os.path.join(config_path, "laser_filter.yaml")).read()
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

    def configured(source_file):
        return ParameterFile(
            RewrittenYaml(
                source_file=source_file, root_key=namespace, param_rewrites={}, convert_types=True
            ),
            allow_substs=True,
        )

    configured_params = configured(params_file)
    configured_laser_filter_params = configured(LaunchConfiguration("laser_filter_params_file"))

    params_filename = PythonExpression(["'nav2_' + '", controller, "' + '.yaml'"])
    declare_args = [
        DeclareLaunchArgument(
            "config_dir",
            default_value="",
            description="Path to a writable copy of the config trees, as produced by "
            "`ros2 run rosbot_utils create_config_dir <dst>`. Empty reads the package share.",
        ),
        DeclareLaunchArgument(
            "controller",
            default_value="mppi",
            description="Nav2 controller type",
            choices=["dwb", "mppi", "rpp"],
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logging level",
            choices=["debug", "info", "warning", "error"],
        ),
        DeclareLaunchArgument(
            "map", default_value="", description="Full path to map yaml file to load"
        ),
        DeclareLaunchArgument(
            "map_save_path",
            default_value=os.path.join(os.path.expanduser("~"), "maps", "map"),
            description="Where map_autosaver writes the SLAM map, without extension "
            "(.yaml/.png are appended). The directory is created if missing.",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
            description="Add namespace to all launched nodes",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([pkg_config_path, params_filename]),
            description="Path to the controller-specific nav2 parameters file",
        ),
        DeclareLaunchArgument(
            "common_params_file",
            default_value=PathJoinSubstitution([pkg_config_path, "nav2_common.yaml"]),
            description="Path to the common nav2 parameters file (shared across controllers)",
        ),
        DeclareLaunchArgument(
            "robot_model",
            default_value=EnvironmentVariable("ROBOT_MODEL", default_value=""),
            description="Specify robot model",
            choices=["rosbot", "rosbot_xl"],
        ),
        DeclareLaunchArgument(
            "preflight",
            default_value="True",
            description="Check that the robot is navigable (lidar, transforms, driver) and "
            "refuse to start nav2 if it is not",
        ),
        DeclareLaunchArgument(
            "preflight_timeout",
            default_value="20.0",
            description="Seconds each preflight check waits before reporting a failure",
        ),
        DeclareLaunchArgument("slam", default_value="True", description="Whether run a SLAM"),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        ),
    ]

    # Everything below runs inside the robot's namespace with the global TF topics
    # remapped onto it. Scoped in a GroupAction (rather than pushed at launch top level)
    # so the stack can be deferred behind the preflight gate without losing the scope.
    def namespaced(*actions, condition=None):
        return GroupAction(
            [
                PushROSNamespace(namespace),
                SetParameter(name="use_sim_time", value=use_sim_time),
                SetRemap("/diagnostics", "diagnostics"),
                SetRemap("/tf", "tf"),
                SetRemap("/tf_static", "tf_static"),
                *actions,
            ],
            condition=condition,
        )

    preflight_node = Node(
        name="autonomy_preflight",
        namespace="",
        package="rosbot_navigation",
        executable="autonomy_preflight",
        arguments=[
            "--namespace",
            namespace,
            "--controller",
            controller,
            "--timeout",
            preflight_timeout,
            "--use-sim-time",
            use_sim_time,
        ],
        output="screen",
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
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
        ]
    )

    def on_preflight_exit(event, _context):
        if event.returncode == 0:
            return [namespaced(bringup_group)]
        return [
            LogInfo(
                msg="\nautonomy preflight failed — not starting nav2. "
                "Fix the items above and try again.\n"
            ),
            EmitEvent(event=Shutdown(reason="autonomy preflight failed")),
        ]

    actions = [
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        *declare_args,
        prepare_params_action,
        # With the gate on, nav2 only starts once preflight reports the robot is
        # navigable; otherwise the whole launch shuts down with a named reason instead
        # of leaving a full nav2 stack spinning against a robot that cannot move.
        GroupAction(
            [
                namespaced(preflight_node),
                RegisterEventHandler(
                    OnProcessExit(target_action=preflight_node, on_exit=on_preflight_exit)
                ),
            ],
            condition=IfCondition(preflight),
        ),
        namespaced(bringup_group, condition=UnlessCondition(preflight)),
    ]

    return LaunchDescription(actions)
