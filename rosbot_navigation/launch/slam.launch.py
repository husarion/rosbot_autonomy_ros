# Copyright (c) 2020 Samsung Research Russia
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node, SetParameter, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Input parameters declaration
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    # Variables
    lifecycle_nodes = ["slam_toolbox", "map_saver"]

    # Getting directories and launch-files
    bringup_dir = get_package_share_directory("nav2_bringup")

    # Create our own temporary YAML files that include substitutions
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Declare the launch arguments
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    # Nodes launching commands
    map_server = GroupAction(
        actions=[
            SetParameter("use_sim_time", use_sim_time),
            Node(
                package="nav2_map_server",
                executable="map_saver_server",
                output="screen",
                respawn_delay=2.0,
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[configured_params],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_slam",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[{"autostart": True}, {"node_names": lifecycle_nodes}],
            ),
        ]
    )

    # slam_toolbox inlined (instead of including online_sync_launch.py) so it gets
    # params via RewrittenYaml(root_key=namespace) — the YAML key stays a plain
    # 'slam_toolbox:'. use_lifecycle_manager lets lifecycle_manager_slam drive it.
    slam_toolbox = GroupAction(
        actions=[
            # Remapping required to have a slam session subscribe & publish in optional namespaces
            SetRemap(src="/scan", dst="scan"),
            SetRemap(src="/tf", dst="tf"),
            SetRemap(src="/tf_static", dst="tf_static"),
            SetRemap(src="/map", dst="map"),
            LifecycleNode(
                package="slam_toolbox",
                executable="sync_slam_toolbox_node",
                name="slam_toolbox",
                namespace="",
                output="screen",
                parameters=[
                    configured_params,
                    {"use_sim_time": use_sim_time, "use_lifecycle_manager": True},
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),  # namespace="" inherits the pushed ROS namespace (like map_saver)
        ]
    )

    actions = [
        declare_namespace_arg,
        declare_params_file_arg,
        declare_use_sim_time_arg,
        declare_log_level_arg,
        map_server,
        slam_toolbox,
    ]

    return LaunchDescription(actions)
