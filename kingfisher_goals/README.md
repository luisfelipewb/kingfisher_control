# kingfisher_goals

Goals for the controllers. Two nodes:

- `goal_publisher` turns a goal in any frame into a stream in `kingfisher/base_link`, and owns when a
  goal ends: reached, or timed out. Controllers such as `kingfisher_rl`'s agents act once per goal
  message and know nothing about goals ending, so when the stream stops, their commands stop and the
  MCU (or `cmd_drive_translate` in sim) zeroes the thrusters.
- `detection_goal` sends the closest detection as a goal. It is the ROS 1 `kingfisher_rl/goal_publisher.py`.

## Running

The controller launches start them (`kingfisher_rl`'s `passthrough_thrusters.launch.py`). On their own:

```bash
ros2 run kingfisher_goals goal_publisher --ros-args -r __ns:=/kingfisher \
  --params-file $(ros2 pkg prefix kingfisher_goals)/share/kingfisher_goals/config/goals.yaml
ros2 topic pub --once /kingfisher/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 5.0, y: 2.0}}}"
```

RViz's goal tool works too, with its topic set to `/kingfisher/goal`.

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

A goal is transformed into `odom`'s frame once, when it arrives, and a new goal replaces it.
`control/goal` is 2D: the position and the heading relative to the boat, stamped with the `odom` it
was computed from. It stops while `odom` is stale.

## Parameters

Defaults in `config/goals.yaml`.

| node | parameter | default | |
|---|---|---|---|
| goal_publisher | `rate` | 10.0 | Hz, the rate the field policy ran at |
| goal_publisher | `reach_distance` | 0.1 | m |
| goal_publisher | `goal_timeout` | 30.0 | s since the goal was received (not its stamp) |
| goal_publisher | `odom_timeout` | 1.0 | s |
| detection_goal | `max_goal_distance` | 5.0 | m from the cloud's frame origin |
