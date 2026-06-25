# scrubber_sim

ROS 2 **Jazzy** + **Gazebo Harmonic (gz-sim)** simulation for a large **three-wheel Ackermann floor-scrubbing robot** (autonomous cleaning vehicle). Full perception–navigation stack: SLAM mapping + Nav2 path planning + a custom behavior tree tuned for Ackermann kinematics.

> 大型三轮阿卡曼洗地车的自动驾驶仿真。ROS 2 Jazzy + Gazebo Harmonic，SLAM 建图 + Nav2 导航 + 阿卡曼定制行为树。

## Robot

- Body: 1.4 × 1.0 × 1.3 m, mass ~200 kg, three-wheel Ackermann (single front wheel: drive + steer)
- Wheel radius 0.15 m, wheelbase 1.0 m, track 0.9 m, min turning radius 1.5 m
- Sensors: 360° 2D LiDAR (gpu_lidar), IMU
- Drive: front-wheel steer + rear-wheel drive (no reversing, no in-place rotation)

## Package layout

```
scrubber_sim/
├── urdf/scrubber.urdf.xacro     # robot model + gz-sim plugins (AckermannSteering / lidar / imu)
├── worlds/simple_world.sdf      # walled room + box obstacles
├── config/
│   ├── nav2.yaml                # Nav2 params (SmacPlannerHybrid + RegulatedPurePursuit)
│   ├── slam_toolbox.yaml        # online async mapping
│   └── ackermann_bt.xml         # custom BT: Spin/BackUp removed (Ackermann can't)
├── launch/
│   ├── bringup.launch.py        # sim + robot + ros_gz bridge
│   ├── slam.launch.py           # bringup + slam_toolbox
│   └── nav2.launch.py           # bringup + slam + Nav2 stack
└── scrubber_sim/
    └── wheel_odometry.py        # custom bicycle-model odometry (see Notes)
```

## Run

```bash
# 1. build
cd ros2_ws && colcon build --symlink-install && source install/setup.bash

# 2. bring up sim + robot
ros2 launch scrubber_sim bringup.launch.py

# 3. mapping
ros2 launch scrubber_sim slam.launch.py
#    drive around with: ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 4. full navigation
ros2 launch scrubber_sim nav2.launch.py
#    send a goal in RViz, or:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

## Notes / design decisions

These are hard-won lessons from getting the stack to actually run on ROS 2 Jazzy + gz-sim:

- **Ackermann BT**: the default Nav2 BT references the `Spin` recovery, which an Ackermann vehicle cannot perform. `config/ackermann_bt.xml` removes `Spin`/`BackUp`, keeping only `Wait` + `DriveOnHeading`. The `behavior_server` plugin list must match.
- **Planner**: `SmacPlannerHybrid` with `motion_model_for_search: DUBIN` (forward-only) + `RegulatedPurePursuitController` with `use_rotate_to_heading: false`, `allow_reversing: false`.
- **Plugin names**: Jazzy uses `::` (e.g. `nav2_smac_planner::SmacPlannerHybrid`), not the `/` form from older releases.
- **Lifecycle managers**: slam_toolbox and the Nav2 nodes are managed by **separate** `nav2_lifecycle_manager` instances — mixing them causes a bond timeout.
- **Custom odometry** (`wheel_odometry.py`): the gz `AckermannSteering` system's built-in odometry is unreliable for this single-front-wheel three-wheel layout (it overshoots/drifts even with correct geometry params). We instead integrate a bicycle model from `/joint_states` (rear-wheel velocity → linear speed, steering-joint angle → yaw rate). This matches the real vehicle's STM32 wheel-odometry, so sim and hardware share the same model.
- **gz launch** must pass `-r` (run), otherwise the sim starts paused and no sensor data flows.

## Tools

Standalone Python helpers under `tools/scrubber-sim/` (no ROS 2 runtime needed, run + test locally):

- **`map_to_polygons/`** — SLAM `map.pgm` → obstacle polygon list (F2C inner voids) via `cv2.findContours` + `approxPolyDP` + pixel→world transform. Feeds the complete-coverage goal so the planner routes *around* mapped static obstacles.
- **`coverage_meter/`** — actual coverage measurement by **footprint sweep** (not path-length estimate). Stamps the cleaning footprint along the recorded trajectory onto a grid, intersects with the cleanable area (outer − voids), and reports coverage %, swept-obstacle area (collision alarm) and overspray area, with a visualization.

```bash
cd tools/scrubber-sim
python3 -m pytest tests/ -q     # 28 tests
```

## Status

Mapping closed-loop + single-goal navigation work end to end. Complete-coverage (boustrophedon) path planning is integrated via F2C + opennav_coverage; current work is the two-layer obstacle-avoidance architecture (static voids upstream + collision-detection/recovery downstream).

## License

MIT
