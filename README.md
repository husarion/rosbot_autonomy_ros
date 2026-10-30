# rosbot-autonomy

Autonomous navigation & mapping for ROSbot with a web user interface powered by Foxglove. Works over the Internet thanks to Husarnet VPN

![autonomy-result](https://github-readme-figures.s3.eu-central-1.amazonaws.com/rosbot/rosbot-autonomy/rosbot-autonomy.webp)

## 🛠️ Setup Repository

### Create Workspace

```bash
mkdir rosbot_autonomy_ws
cd rosbot_autonomy_ws
git clone -b jazzy https://github.com/husarion/rosbot_autonomy_ros.git src/rosbot_autonomy_ros
```

### Build

```bash
sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Run

```bash
source install/setup.bash
ros2 launch rosbot_navigation bringup.launch.py robot_model:=<rosbot/rosbot_xl>
```

> [!NOTE]
> Additional arguments are detailed in the [Launch Arguments](#launch-arguments) section.
> MPPI controller is not compatible with **ROSbot 2 PRO**. Please use DWB or RPP controller.

## 🚀 Demo

### 📋 Requirements

1. **ROSbot Platform & ROS Driver**

    This demo is prepared for the **ROSbot Series** (ROSbot XL, ROSbot 3 / 3 PRO). This version is prepared to work with [rosbot](https://snapcraft.io/rosbot) ROS driver snap. To install snap follow the information in snapcraft.

    The driver must run `twist_mux_controller` (rosbot_ros ≥ 1.2.0) — Nav2 publishes to `autonomous/cmd_vel`, not `cmd_vel` (see [Velocity command arbitration](#velocity-command-arbitration)). On an older driver the robot will not move; override `collision_monitor.cmd_vel_out_topic` back to `cmd_vel` through `common_params_file` if you cannot update it.

2. **Robot Configuration**

    The demo assumes that the `scan` topic (`LaserScan` message type) is available.

3. **Just**

    To simplify running commands, we use [just](https://github.com/casey/just). Install it with:

    ```bash
    sudo snap install just
    ```

4. **DDS**

    The default configuration starts [FastDDS - UDP](docker/dds-config-udp.xml) configuration. All snap should share the same DDS configuration.

### 🧭 Navigation

#### Step 1: Environment configuration

Setup environment variables in `docker/.env`: `ROBOT_MODEL` (`rosbot` / `rosbot_xl`), `ROBOT_NAMESPACE` (must match the driver's namespace), `CONFIGURATION`, `CONTROLLER` (`dwb` / `mppi` / `rpp`) and `SLAM` (`True` / `False`).

In simulation, Nav2 needs a lidar publishing the `scan` topic, so pick a `CONFIGURATION` that includes one:

| `ROBOT_MODEL` | Configurations with a lidar                      |
| ------------- | ------------------------------------------------ |
| `rosbot`      | `basic`                                          |
| `rosbot_xl`   | `autonomy`, `manipulation`, `manipulation_pro`   |

On `rosbot_xl`, `basic` and `telepresence` have no lidar and will not work with Nav2.

#### Step 2: Run navigation

Run navigation on the **physical robot**:

```bash
just start-navigation
```

Run navigation in **Gazebo simulation**:

```bash
just start-simulation
```

#### Step 3: Control the robot from a Web Browser

1. Install and run husarion-webui

    ```bash
    just start-visualization
    ```

2. Open the your browser on your laptop and navigate to:

    - http://{ip_address}:8080/ui (devices in the same LAN)
    - http://{hostname}:8080/ui (devices in the same Husarnet Network)

## Documentation

### Startup checks

Nav2 starts happily against a half-configured robot and then simply does nothing — a
missing lidar, a missing transform to the laser frame and a driver that is not running
all look identical from the outside. `autonomy_preflight` runs first and names the
problem:

```
autonomy preflight — namespace (none), timeout 20 s

  [ok  ] driver TF      odom -> base_link is available
  [ok  ] odometry       /odometry/filtered is publishing
  [ok  ] velocity sink  the driver listens on /autonomous/cmd_vel
  [ok  ] lidar          /scan is publishing (frame_id 'laser')
  [FAIL] laser TF       no transform base_link -> laser

--- laser TF ---
The scan is published in frame 'laser', but that frame is not in the robot's
URDF, so nav2 cannot place the measurements. The robot configuration most
likely has no lidar mounted.
    sudo snap set rosbot driver.configuration=autonomy
    sudo rosbot.restart
```

Nav2 is not started unless every check passes; the launch shuts down instead. Fix hints
adapt to how the robot is installed (snap commands vs `ros2 launch`).

Run it on its own against a live robot:

```bash
ros2 run rosbot_navigation autonomy_preflight --namespace my_robot
```

For a UI, `--format jsonl` prints one JSON object per line and flushes each check as
soon as it finishes:

```json
{"type": "begin", "namespace": "my_robot", "timeout": 30.0}
{"type": "check", "name": "driver TF", "ok": true, "detail": "odom -> base_link is available", "fix": null}
{"type": "end", "ok": true}
```

Disable the gate with `preflight:=False`, or give slow hardware more room with
`preflight_timeout:=40.0`.

rosbot_ros bridges `/<namespace>/tf` onto the global `/tf`, rewriting frame_ids with a
`tf_prefix` along the way. Plain frame names like `base_link`/`odom` therefore only ever
exist on the namespaced `/<namespace>/tf` topic, never on the bridged global one —
`autonomy_preflight` remaps `/tf`(`_static`) to the relative topic itself so its checks
keep working for a namespaced robot, the same way `bringup.launch.py` does for the rest
of the nav2 stack via `SetRemap`.

### Velocity command arbitration

The driver arbitrates between velocity sources inside its control loop
(`twist_mux_controller`), so navigation and a human operator can be connected at
the same time:

| Topic | Priority | Published by |
| ----- | -------- | ------------ |
| `manual/cmd_vel` | 100 | gamepad, keyboard teleop, the Foxglove joystick panel |
| `autonomous/cmd_vel` | 10 | Nav2 (`collision_monitor` output) |
| `cmd_vel` | 1 | anything else |

The highest-priority source that published within the last 0.2 s wins. Grabbing
the gamepad overrides navigation immediately, and letting go hands control back
— no mode switch. `twist_mux_controller/source` reports who is driving.

### Erasing stale walls (SLAM)

slam_toolbox re-renders its grid from every scan it has kept, and a cell stays
occupied while hits / pass-throughs > `occupancy_threshold` (0.2 here, karto's
default is 0.1). A wall that is gone therefore fades only after the robot has
looked through it several times more often than it saw it.

With `slam:=True`, slam_toolbox publishes `slam_map` and `map_eraser` republishes
it as `map` / `map_metadata`, which Nav2, `map_autosaver` and every viewer read:

| Interface | Type | Effect |
| --------- | ---- | ------ |
| `map_eraser/erase` | `nav2_msgs/srv/ClearCostmapAroundPose` | Cells within `reset_distance` (m, ≤ `max_radius`, default 5) of `pose` (frame `map`) that slam still calls occupied are published as unknown. The first time slam calls a cell free it is released, so a real obstacle that appears there later shows again. |
| `map_eraser/reset` | `std_srvs/srv/Trigger` | Drops every erasure. |

Erasures live in the node's memory: a restart of navigation drops them, but a map
saved meanwhile keeps them.

**Automatic clearing** (`map_eraser.auto_clear.enabled`, on in
[`nav2_common.yaml`](./rosbot_navigation/config/nav2_common.yaml)) does the same
without anyone pressing anything: `map_eraser` traces the live `scan_filtered`
beams and publishes a cell slam still calls occupied as unknown once beams have
kept crossing it (`min_free` decayed pass-throughs, time constant `decay`) and no
beam has ended in it, or within `hit_radius_cells` of it, for `hit_hold` seconds.
A beam's last `hit_margin` + `hit_margin_per_m` × range is not counted, and a beam
with no return counts as free up to `range` — karto ignores those, so a wall that
stood in front of open space never fades in slam itself. The first beam that ends
in the cell again shows the obstacle at the next map update.

In the Gazebo sim (box 1.3 m ahead, mapped over 10 drives of ~0.7 m, then
removed): slam alone still held 24 of its 37 cells after 12 more drives; `map`
showed none after 6 (~30 s, `hit_hold`), and the box was back in `map` at the
first update after it reappeared. Along walls the clearing eats the 1–2 cells
slam drew too thick, never the wall itself.

`slam_executable:=lifelong_slam_toolbox_node` (experimental upstream) makes slam
itself forget: it drops older graph nodes that newer scans overlap. With upstream's
lifelong parameters it removed nothing in the sim run; with the ones in
`nav2_common.yaml` the same box went from 40 cells to 15 after 8 drives in
`slam_map` (the rest came from the first two nodes, which lifelong never drops),
and after ~100 removed nodes the walls of the explored area were still whole.
Sync slam with automatic clearing stays the default.

### Launch Arguments

| Argument         | Description <br/> ***Type:*** `Default`                                                               |
| ---------------- | ----------------------------------------------------------------------------------------------------- |
| `common_params_file` | Path to the common nav2 parameters file (merged with `params_file`). <br/> ***string:*** [`nav2_common.yaml`](./rosbot_navigation/config/nav2_common.yaml) |
| `config_dir`     | Writable copy of the config trees, as produced by `ros2 run rosbot_utils create_config_dir <dst> --add rosbot_navigation:config`. Empty reads the package share. <br/> ***string:*** `''` |
| `controller`     | Nav2 controller type. <br/> ***string*** `mppi` (choices: `dwb`, `mppi`, `rpp`)                        |
| `log_level`      | Logging level. <br/> ***string*** `info` (choices: `debug`, `info`, `warning`, `error`)               |
| `map`            | Path to map yaml file to load. Required with `slam:=False` — the launch fails if the file does not exist. <br/> ***string:*** `''` |
| `map_save_path`  | Where `map_autosaver` writes the SLAM map, without extension (`.yaml`/`.png` appended). The directory is created if missing. <br/> ***string:*** `~/maps/map` |
| `namespace`      | Add namespace to all launched nodes. <br/> ***string:*** `env(ROBOT_NAMESPACE)`                       |
| `preflight`      | Check that the robot is navigable (lidar, transforms, driver) and refuse to start nav2 if it is not. <br/> ***bool:*** `True` |
| `preflight_timeout` | Seconds each preflight check waits before reporting a failure. <br/> ***float:*** `20.0` |
| `params_file`    | Path to the controller-specific nav2 parameters file. <br/> ***string:*** [`nav2_<controller>.yaml`](./rosbot_navigation/config/) |
| `robot_model`    | Specify robot model. <br/> ***string:*** `env(ROBOT_MODEL)` (choices: `rosbot`, `rosbot_xl`)          |
| `slam`           | Whether run a SLAM. <br/> ***bool:*** `True`                                                          |
| `slam_executable` | slam_toolbox node for `slam:=True`: `sync_slam_toolbox_node` or `lifelong_slam_toolbox_node`. <br/> ***string:*** `sync_slam_toolbox_node` |
| `use_sim_time`   | Use simulation (Gazebo) clock if true. <br/> ***bool:*** `False`                                      |
