# lidar_slam_ws — two-laptop distributed SLAM

A ROS 2 Jazzy workspace that splits lidar SLAM across two laptops on the same
WiFi network.

```
        LAPTOP 1  (lidar)                      LAPTOP 2  (control station)
  ┌──────────────────────────┐            ┌──────────────────────────────────┐
  │ ydlidar_ros2_driver      │            │ icp_odom_node                    │
  │   → /scan                │  ── WiFi ─▶│   /scan → /odom, odom→base_link  │
  │ static_transform_pub     │   /scan    │ slam_toolbox                     │
  │   base_link→laser_frame  │    +TF     │   → /map, map→odom               │
  │                          │            │ rviz2                            │
  └──────────────────────────┘            └──────────────────────────────────┘
```

Only the raw scan crosses the network. The map is built and rendered on laptop 2
and never leaves it, which keeps WiFi traffic to a few hundred kB/s.

**TF chain while running**

| transform | published by | machine |
|---|---|---|
| `map → odom` | slam_toolbox | laptop 2 |
| `odom → base_link` | icp_odom_node | laptop 2 |
| `base_link → laser_frame` | static_transform_publisher | laptop 1 |

---

## Packages

| package | purpose |
|---|---|
| `lidar_slam_bringup` | launch files, lidar/SLAM/RViz configs, DDS profile template |
| `laser_scan_odometry` | point-to-line ICP scan matcher producing `odom → base_link` |

Why `laser_scan_odometry` exists: slam_toolbox will not start mapping without an
`odom → base_link` transform, and a lidar with no wheel encoders has nothing to
produce one. This node derives it from the scans themselves. It is dead
reckoning and it drifts; slam_toolbox absorbs that drift into `map → odom`.

---

## Setup

### 1. Control station (laptop 2 — this machine)

```bash
cd ~/lidar_slam_ws
./setup/setup_control_station.sh          # apt deps + colcon build, needs sudo
```

### 2. Lidar laptop (laptop 1)

Copy the workspace over, then run its setup script:

```bash
rsync -av --exclude build --exclude install --exclude log \
      ~/lidar_slam_ws/ user@lidar-laptop:~/lidar_slam_ws/

# on laptop 1
cd ~/lidar_slam_ws
./setup/setup_lidar_laptop.sh             # builds YDLidar-SDK + driver, needs sudo
```

`ydlidar_ros2_driver` is not packaged for Jazzy, so the script clones and builds
both it and the YDLidar SDK. It also installs a udev rule giving a stable
`/dev/ydlidar` symlink — log out and back in once afterwards so the `dialout`
group membership takes effect.

### 3. Network (both laptops)

Find each machine's WiFi address with `hostname -I`, then on **each** laptop
point it at the **other**:

```bash
# on laptop 2, where 192.168.1.50 is laptop 1
./setup/make_dds_config.sh 192.168.1.50

# on laptop 1, where 192.168.1.51 is laptop 2
./setup/make_dds_config.sh 192.168.1.51
```

Then, in **every** shell that runs ROS on **either** machine:

```bash
source ~/lidar_slam_ws/setup/ros_network_env.sh
```

This sets `ROS_DOMAIN_ID=42`, selects Cyclone DDS, and points it at the
generated profile. Both machines must agree on the domain ID and the RMW
implementation or they will never see each other.

> **Why a custom DDS profile?** ROS 2 discovery normally uses IP multicast, and
> consumer WiFi access points routinely drop, rate-limit, or refuse to bridge
> multicast between wireless clients. The classic symptom is two laptops that
> ping each other perfectly but never see each other's topics. The generated
> profile names both peers explicitly, making discovery pure unicast.

---

## Running

**Laptop 1:**

```bash
source ~/lidar_slam_ws/setup/ros_network_env.sh
ros2 launch lidar_slam_bringup lidar_laptop.launch.py lidar_model:=x4
```

**Laptop 2:**

```bash
source ~/lidar_slam_ws/setup/ros_network_env.sh
ros2 launch lidar_slam_bringup control_station.launch.py
```

RViz opens with the map, live scan, and TF tree already configured. Walk the
lidar slowly around the space and the map fills in.

**Save the map** when you are happy with it:

```bash
./setup/save_map.sh my_office        # → maps/my_office.{pgm,yaml,posegraph}
```

### Launch arguments worth knowing

`lidar_laptop.launch.py`

| argument | default | notes |
|---|---|---|
| `lidar_model` | `x4` | `x2`, `x4`, or `g2` — picks the matching `config/ydlidar_*.yaml` |
| `port` | `/dev/ydlidar` | use `/dev/ttyUSB0` if you skipped the udev rule |
| `laser_x/y/z` | `0,0,0.18` | where the lidar sits on the platform, in metres |
| `laser_yaw` | `0.0` | set this if the lidar's 0° mark does not point forward |

`control_station.launch.py`

| argument | default | notes |
|---|---|---|
| `odometry` | `icp` | `icp` (scan matching), `static` (identity), `external` (laptop 1 supplies it) |
| `rviz` | `true` | `false` runs the mapping stack headless |
| `publish_laser_tf` | `false` | set `true` if laptop 1 publishes `/scan` but no TF |
| `slam_params_file` | `config/slam_toolbox_async.yaml` | |

### Matching a domain/RMW already in use

`ros_network_env.sh` defaults to domain 42 and Cyclone DDS. If the other laptop
is already set up differently, override rather than reconfigure both machines:

```bash
LIDAR_SLAM_DOMAIN=10 LIDAR_SLAM_RMW=rmw_fastrtps_cpp \
  source ~/lidar_slam_ws/setup/ros_network_env.sh
```

The unicast-peer DDS profile applies to Cyclone only; FastDDS is fine as long as
discovery is already working across the two machines.

### Bench test on one machine first

```bash
ros2 launch lidar_slam_bringup single_machine.launch.py lidar_model:=x4
```

If mapping works here but not across the two laptops, the problem is the
network, not the SLAM configuration. Worth establishing before debugging DDS.

---

## Troubleshooting

Run the diagnostic first — it checks environment, reachability, firewall, the
ROS graph, and the TF chain, in the order these things actually break:

```bash
./setup/check_link.sh <other-laptop-ip>
```

| symptom | likely cause |
|---|---|
| Laptops ping fine, no topics visible | WiFi multicast, or the DDS profile is missing on one side. Re-run `make_dds_config.sh` on **both**. |
| `/scan` listed but `ros2 topic hz /scan` shows nothing | QoS mismatch, or the AP has client isolation enabled — that blocks laptop-to-laptop traffic entirely. |
| RViz shows "No transform from [laser_frame] to [map]" | laptop 1's static transform publisher is not running, or slam_toolbox has not received enough scans to start. |
| Map builds but smears or double-walls when turning | `laser_yaw` is wrong, or the platform is moving faster than the matcher can follow. Move more slowly. |
| `scan match rejected` warnings streaming | Too few features (a bare corridor), scans arriving too slowly over WiFi, or `max_range` set beyond the sensor's real range. |
| Odometry drifts steadily but the map stays sane | Normal. That is exactly the drift slam_toolbox is correcting. |

**Tuning knobs.** `config/icp_odometry.yaml` documents each matcher parameter.
The two that matter most: `keyframe_distance`/`keyframe_angle` control how often
the reference scan is replaced (tighter = more robust, more drift), and
`max_mean_error`/`min_inlier_ratio` control how readily a match is rejected.

---

## Tests

```bash
cd ~/lidar_slam_ws/src/laser_scan_odometry
python3 -m pytest test/ -q
```

The suite runs the matcher against a simulated room with no ROS graph involved:
SE(2) algebra identities, motion recovery for known transforms, rejection of
unmatchable scans, and a 200-frame trajectory that must track a 4 m drive with
2 rad of rotation to within 5 cm without a single failed match.
