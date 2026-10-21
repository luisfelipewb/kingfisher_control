# kingfisher_goals

Goals for the controllers. Four nodes:

- `goal_publisher` turns a goal in any frame into a stream in `kingfisher/base_link`, and owns when a
  goal ends: reached, or timed out. Controllers such as `kingfisher_rl`'s agents act once per goal
  message and know nothing about goals ending, so when the stream stops, their commands stop and the
  MCU (or `cmd_drive_translate` in sim) zeroes the thrusters.
- `detection_goal` sends the closest detection as a goal. It is the ROS 1 `kingfisher_rl/goal_publisher.py`.
- `waypoint_goal` follows a list of waypoints and sends the current one as the goal. A detection near
  the current waypoint replaces it until the detections stop, so the boat picks up bottles on the way.
  It replaces the ROS 1 `kingfisher_experiments` `waypoint_publisher` and its `goal_mux`.
- `virtual_detections` publishes virtual bottles as a detector would see them, and drops each one
  once the boat collects it. It runs the same in sim and on the water, and replaces `pimage_utils`'
  `simulated_waste` and `fake_detector`.

```
waypoints ─► waypoint_goal ─► goal ─► goal_publisher ─► control/goal ─► agent
detections ─┘  (from virtual_detections or a detector)
```

## Running

The controller launches start them (`kingfisher_rl`'s `passthrough_thrusters.launch.py`). On their own:

```bash
ros2 run kingfisher_goals goal_publisher --ros-args -r __ns:=/kingfisher \
  --params-file $(ros2 pkg prefix kingfisher_goals)/share/kingfisher_goals/config/goals.yaml
ros2 topic pub --once /kingfisher/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 5.0, y: 2.0}}}"
```

RViz's goal tool works too, with its topic set to `/kingfisher/goal`.

### Waypoints

`launch/waypoints.launch.py` runs `waypoint_goal` on a scenario, plus `virtual_detections` on the
same scenario with `virtual_detections:=true`. Run it next to a controller launch:

```bash
ros2 launch kingfisher_rl passthrough_thrusters.launch.py                   # use_sim_time:=true in sim
ros2 launch kingfisher_goals waypoints.launch.py scenario:=grid_small virtual_detections:=true
```

Then press **A** to hand `cmd_drive` to the agent. A scenario is a name under `config/scenarios/` or a
path to a YAML file:

```yaml
frame_id: launch_site          # any frame in TF
waypoints:
  - [10, 0]
  - [20, 0]
bottles:                       # for virtual_detections
  - [12.3, 1.7]
```

`config/scenarios/` holds the ROS 1 grids, with their offset baked in and `world` replaced by
`launch_site`. A list published on `waypoints` replaces the current one at any time:

```bash
ros2 topic pub --once /kingfisher/waypoints geometry_msgs/msg/PoseArray \
  "{header: {frame_id: launch_site}, poses: [{position: {x: 10.0}}, {position: {x: 20.0, y: 5.0}}]}"
```

## Interface

Relative to the node's namespace (`/kingfisher`).

| | name | type | node |
|---|---|---|---|
| sub | `goal` | `geometry_msgs/PoseStamped`, any frame in TF | goal_publisher |
| sub | `odom` | `nav_msgs/Odometry` | goal_publisher |
| pub | `control/goal` | `PoseStamped` in `odom`'s child frame, at `rate` while a goal is active | goal_publisher |
| pub | `control/status` | `std_msgs/Bool`, true while a goal is active, at `rate` | goal_publisher |
| sub | `detections` | `sensor_msgs/PointCloud2` | detection_goal |
| pub | `goal` | `PoseStamped`, the cloud's header | detection_goal |
| sub | `waypoints` | `geometry_msgs/PoseArray`, any frame | waypoint_goal |
| sub | `detections` | `PointCloud2`, any frame | waypoint_goal |
| sub | `odom` | `Odometry` | waypoint_goal, virtual_detections |
| pub | `goal` | `PoseStamped` in `odom`'s frame, at `rate` until the last waypoint | waypoint_goal |
| pub | `waypoints/active` | `PoseArray` in `odom`'s frame, latched | waypoint_goal |
| pub | `detections` | `PointCloud2` in `odom`'s child frame, at `rate`; the bottles in view | virtual_detections |
| pub | `virtual_detections/bottles` | `PointCloud2` in `odom`'s frame, at `rate`; all remaining | virtual_detections |

A goal is transformed into `odom`'s frame once, when it arrives, and a new goal replaces it.
`control/goal` is 2D: the position and the heading relative to the boat, stamped with the `odom` it
was computed from. It stops while `odom` is stale.

`waypoint_goal` moves to the next waypoint within `reach_distance` and stops after the last one, so
`goal_publisher` finishes on it. The detection it takes is the one closest to the boat within
`capture_radius` of the current waypoint, while one came in the last `detection_timeout`.
`virtual_detections` sees the bottles within `max_range` and `half_fov_deg` of the bow; there is no
camera model, noise or false negatives.

## Parameters

Defaults in `config/goals.yaml`.

| node | parameter | default | |
|---|---|---|---|
| goal_publisher | `rate` | 10.0 | Hz, the rate the field policy ran at |
| goal_publisher | `reach_distance` | 0.1 | m |
| goal_publisher | `goal_timeout` | 30.0 | s since the goal was received (not its stamp) |
| goal_publisher | `odom_timeout` | 1.0 | s |
| detection_goal | `max_goal_distance` | 5.0 | m from the cloud's frame origin |
| waypoint_goal | `waypoints_file` | `''` | scenario YAML; empty: wait for `waypoints` |
| waypoint_goal | `rate` | 2.0 | Hz |
| waypoint_goal | `reach_distance` | 2.0 | m, the ROS 1 `dist_threshold` |
| waypoint_goal | `capture_radius` | 7.0 | m, the ROS 1 `goal_bounds_threshold` |
| waypoint_goal | `detection_timeout` | 1.0 | s |
| waypoint_goal | `loop` | false | start over after the last waypoint |
| virtual_detections | `bottles_file` | — | scenario YAML |
| virtual_detections | `rate` | 2.0 | Hz, the ROS 1 detection rate |
| virtual_detections | `collect_radius` | 0.33 | m |
| virtual_detections | `max_range` | 10.0 | m; to check against the camera |
| virtual_detections | `half_fov_deg` | 35.0 | deg; to check against the camera |
