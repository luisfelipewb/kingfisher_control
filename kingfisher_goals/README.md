# kingfisher_goals

Goals for the controllers. Five nodes:

- `goal_publisher` turns a goal in any frame into a stream in `kingfisher/base_link`, and owns when a
  goal ends: reached, cleared, or timed out. Controllers such as `kingfisher_rl`'s agents act once per goal
  message and know nothing about goals ending, so when the stream stops, their commands stop and the
  MCU (or `cmd_drive_translate` in sim) zeroes the thrusters.
- `detection_goal` sends the closest detection as a goal. It is the ROS 1 `kingfisher_rl/goal_publisher.py`.
- `waypoint_goal` follows a list of waypoints and sends the current one as the goal. A detection near
  the current waypoint replaces it until the detections stop, so the boat picks up bottles on the way.
  It replaces the ROS 1 `kingfisher_experiments` `waypoint_publisher` and its `goal_mux`.
- `virtual_detections` publishes virtual bottles as a detector would see them, and drops each one
  once the boat collects it. It runs the same in sim and on the water, and replaces `pimage_utils`'
  `simulated_waste` and `fake_detector`.
- `relative_goal` runs a list of goals relative to the boat, keeping them inside a test area, to
  measure a policy over a range of distances and bearings.

```
waypoints ─► waypoint_goal ─► goal ─► goal_publisher ─► control/goal ─► agent
detections ─┘  (from virtual_detections or a detector)
goals file ─► relative_goal ─► goal, goal/clear ─► goal_publisher
                            └─► control/cmd_drive, between goals
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

### Relative goals

`launch/relative_goal.launch.py` runs `relative_goal` on a scenario. Run it next to a controller launch,
with the boat inside the test area:

```bash
ros2 launch kingfisher_rl passthrough_thrusters.launch.py                   # use_sim_time:=true in sim
ros2 launch kingfisher_goals relative_goal.launch.py scenario:=relative_medium
```

Then press **A**. **B** pauses and puts the current goal back in the list, and **A** carries on. The
node reads these presses from `joy`, because `drive_mux` doesn't publish its selection, so a
`drive_mux/select` call by hand isn't seen.

The launch records the run to `~/bags/relative_goal/<scenario>_<date-time>` (`record:=false` to skip,
`bag_dir:=` for another folder). `relative_goal` exits once all goals are done, and the launch with
it, which closes the bag; Ctrl-C closes it too. In the bag, `relative_goal/status` marks each goal as
`goal <index> <distance> <heading>`: its index in the scenario file, m and deg.

Two scripts, not installed, turn a bag into CSVs and plots, with ROS sourced:

```bash
scripts/relative_goal_csv.py ~/bags/relative_goal/relative_medium_2026-11-18_14-05-30
scripts/plot_relative_goals.py ~/bags/relative_goal/relative_medium_2026-11-18_14-05-30.csv
```

The first writes, next to the bag, `<bag>.csv` with a row per `control/cmd_drive` action (the policy's
and the lead-in's) and odom interpolated to it, including the pose in the frame of the boat when the
goal was sent, and `<bag>_goals.csv` with a row per goal sent: time, outcome (reached, missed, put
back), closest range and the start speed and yaw rate. The second plots every goal from its start,
and the whole run in `odom` by phase.

Its scenario is a list in the robot frame, x forward and y to port, and a goal can appear more than once:

```yaml
goals:
  - [3.536, 3.536]
  - [9.848, -1.736]
```

`scripts/generate_goals.py` writes a shuffled grid of distances and bearings. It isn't installed, so run
it from this package's source tree. `relative_grid` and `relative_small` came from it, with the command
on their first line:

```bash
scripts/generate_goals.py --distance 1 10 1 --bearing -45 45 5 --seed 0 \
  --output config/scenarios/relative_grid.yaml
```

`--repeat N` puts each goal in the list N times, and `--seed` sets the order.

## Interface

Relative to the node's namespace (`/kingfisher`).

| | name | type | node |
|---|---|---|---|
| sub | `goal` | `geometry_msgs/PoseStamped`, any frame in TF | goal_publisher |
| sub | `goal/clear` | `std_msgs/Empty`, drops the current goal | goal_publisher |
| sub | `odom` | `nav_msgs/Odometry` | goal_publisher |
| pub | `control/goal` | `PoseStamped` in `odom`'s child frame, at `rate` while a goal is active | goal_publisher |
| pub | `control/status` | `std_msgs/Bool`, true while a goal is active, at `rate` | goal_publisher |
| sub | `detections` | `sensor_msgs/PointCloud2` | detection_goal |
| pub | `goal` | `PoseStamped`, the cloud's header | detection_goal |
| sub | `waypoints` | `geometry_msgs/PoseArray`, any frame | waypoint_goal |
| sub | `detections` | `PointCloud2`, any frame | waypoint_goal |
| sub | `odom` | `Odometry` | waypoint_goal, virtual_detections, relative_goal |
| pub | `goal` | `PoseStamped` in `odom`'s frame, at `rate` until the last waypoint | waypoint_goal |
| pub | `waypoints/active` | `PoseArray` in `odom`'s frame, latched | waypoint_goal |
| pub | `detections` | `PointCloud2` in `odom`'s child frame, at `rate`; the bottles in view | virtual_detections |
| pub | `virtual_detections/bottles` | `PointCloud2` in `odom`'s frame, at `rate`; all remaining | virtual_detections |
| sub | `joy` | `sensor_msgs/Joy`, the buttons that switch `drive_mux` | relative_goal |
| pub | `goal` | `PoseStamped` in `odom`'s frame, once per goal | relative_goal |
| pub | `goal/clear` | `Empty`, when a goal ends | relative_goal |
| pub | `control/cmd_drive` | `kingfisher_msgs/Drive`, at `rate` during a lead-in | relative_goal |
| pub | `relative_goal/active` | `Bool`, at `rate`; true while a goal runs under the policy | relative_goal |
| pub | `relative_goal/status` | `std_msgs/String`, latched: `waiting`, `lead-in`, `recentring`, `goal <index> <distance> <heading>` (`goal 7 3.00 -45.0`), `done` | relative_goal |
| pub | `relative_goal/markers` | `visualization_msgs/MarkerArray` in `area_frame`, latched; the remaining goals, and the selected one in red | relative_goal |

A goal is transformed into `odom`'s frame once, when it arrives, and a new goal replaces it.
`control/goal` is 2D: the position and the heading relative to the boat, stamped with the `odom` it
was computed from. It stops while `odom` is stale.

`waypoint_goal` moves to the next waypoint within `reach_distance` and stops after the last one, so
`goal_publisher` finishes on it. The detection it takes is the one closest to the boat within
`capture_radius` of the current waypoint, while one came in the last `detection_timeout`.
`virtual_detections` sees the bottles within `max_range` and `half_fov_deg` of the bow; there is no
camera model, noise or false negatives.

`relative_goal` places each goal from the boat's pose when it selects it, and takes the first in the file
inside the area, so the file's order is a preference. Every goal starts from a lead-in, so the policy gets
the boat in the same state each time: the node drives it on `control/cmd_drive` with its own speed and
heading controller, holding the heading the boat had and ramping the speed at `lead_in.max_accel`, and sends
the goal once the boat has stayed at `start_speed` and barely turning for `settle_time`. It shares
`control/cmd_drive` with the agent, which only drives while goals stream, and `settle_time` is longer than
the agent's `episode_gap`, so each goal is a new episode. When no goal fits, the lead-in heads for the
area's centre and also waits for the bow to point at it. A goal ends reached or missed, and either way it
is done; a missed one isn't retried. Progress is kept in memory, so a restart runs the whole list again.
The area limits goals, not the boat, which can overshoot a goal near the edge, and it doesn't stop the boat
going straight to the centre from outside the area when **A** is pressed.

## Parameters

Defaults in `config/goals.yaml`.

| node | parameter | default | |
|---|---|---|---|
| goal_publisher | `rate` | 10.0 | Hz, the rate the field policy ran at |
| goal_publisher | `reach_distance` | 0.1 | m |
| goal_publisher | `goal_timeout` | 0.0 | s since the goal was received (not its stamp); 0 for no timeout |
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
| relative_goal | `goals_file` | — | scenario YAML |
| relative_goal | `rate` | 10.0 | Hz |
| relative_goal | `area_frame` | `test_center` | any frame in TF |
| relative_goal | `area_half_size` | 15.0 | m either side of the frame's origin, in x and y |
| relative_goal | `reach_distance` | 0.33 | m, the same as `collect_radius` |
| relative_goal | `goal_timeout` | 30.0 | s before a goal is missed |
| relative_goal | `start_speed` | 0.5 | m/s forward when a goal is sent |
| relative_goal | `start_speed_tolerance` | 0.05 | m/s |
| relative_goal | `start_yaw_rate` | 0.05 | rad/s, at most, when a goal is sent |
| relative_goal | `turn_speed` | 0.2 | m/s, the least while the bow is far off: the boat barely turns standing still |
| relative_goal | `settle_time` | 1.0 | s ready before a goal is sent, longer than the agent's `episode_gap` |
| relative_goal | `recentre_heading` | 10.0 | deg between the bow and the centre, to end recentring |
| relative_goal | `lead_in.*` | see `goals.yaml` | the lead-in controller's gains, tuned in sim only |
| relative_goal | `control_button` | 0 | `joy` button that selects `control/cmd_drive` (A) |
| relative_goal | `teleop_button` | 1 | `joy` button that selects `teleop/cmd_drive` (B) |
