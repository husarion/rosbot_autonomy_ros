# rosbot-autonomy

Autonomous navigation & mapping for ROSbot 2R / 2 PRO with a web user interface powered by Foxglove. Works over the Internet thanks to Husarnet VPN

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

    This demo is prepared for the **ROSbot Series** (ROSbot XL, ROSbot 3 / 3 PRO, ROSbot 2R / 2 PRO). This version is prepared to work with [rosbot](https://snapcraft.io/rosbot) ROS driver snap. To install snap follow the information in snapcraft.

    The driver must run `twist_mux_controller` — Nav2 publishes to `autonomous/cmd_vel`, not `cmd_vel` (see [Velocity command arbitration](#velocity-command-arbitration)). On an older driver the robot will not move; override `collision_monitor.cmd_vel_out_topic` back to `cmd_vel` through `common_params_file` if you cannot update it.

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

Setup environment variable in `docker/.env`.

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

Disable the gate with `preflight:=False`, or give slow hardware more room with
`preflight_timeout:=40.0`.

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

### Launch Arguments

| Argument         | Description <br/> ***Type:*** `Default`                                                               |
| ---------------- | ----------------------------------------------------------------------------------------------------- |
| `common_params_file` | Path to the common nav2 parameters file (merged with `params_file`). <br/> ***string:*** [`nav2_common.yaml`](./rosbot_navigation/config/nav2_common.yaml) |
| `config_dir`     | Writable copy of the config trees, as produced by `ros2 run rosbot_utils create_config_dir <dst> --add rosbot_navigation:config`. Empty reads the package share. <br/> ***string:*** `''` |
| `controller`     | Nav2 controller type. <br/> ***string*** `mppi` (choices: `dwb`, `mppi`, `rpp`)                        |
| `log_level`      | Logging level. <br/> ***string*** `info` (choices: `debug`, `info`, `warning`, `error`)               |
| `map`            | Path to map yaml file to load. Only used with `slam:=False`. <br/> ***string:*** `''`                  |
| `map_save_path`  | Where `map_autosaver` writes the SLAM map, without extension (`.yaml`/`.png` appended). The directory is created if missing. <br/> ***string:*** `~/maps/map` |
| `namespace`      | Add namespace to all launched nodes. <br/> ***string:*** `env(ROBOT_NAMESPACE)`                       |
| `params_file`    | Path to the controller-specific nav2 parameters file. <br/> ***string:*** [`nav2_<controller>.yaml`](./rosbot_navigation/config/) |
| `robot_model`    | Specify robot model. <br/> ***string:*** `env(ROBOT_MODEL)` (choices: `rosbot`, `rosbot_xl`)          |
| `slam`           | Whether run a SLAM. <br/> ***bool:*** `True`                                                          |
| `use_sim_time`   | Use simulation (Gazebo) clock if true. <br/> ***bool:*** `False`                                      |
