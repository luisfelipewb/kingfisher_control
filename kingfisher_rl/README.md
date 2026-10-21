# kingfisher_rl

RL agents for the Kingfisher, as ONNX policies run with `onnxruntime` on the CPU. They are the same in
sim and on the boat.

An agent does one inference and sends one command per `control/goal` message from `kingfisher_goals`'
`goal_publisher`, which owns the goal: its rate (10 Hz, the training step), when it is reached and when
it times out. When goals stop, commands stop, and the MCU (or `cmd_drive_translate` in sim) zeroes the
thrusters.

## Naming

Agents are `<task>_<actuation>`, one executable each. That name is also the node and the module.
Task names are one word, and actuation is `thrusters` or `sail` (thrusters + sail). Policies are
`models/<task>_<actuation>/<tag>.onnx`.

| agent | task | observation (in order) | action |
|---|---|---|---|
| `passthrough_thrusters` | drive through the goal position | previous action (2), `odom` twist vx, vy (2) and wz (1), cos and sin of the goal bearing (2), goal distance capped at 9 m (1) | `control/cmd_drive` left, right |

The observation is kept exactly as in training. `agent_base.py` holds the shared part: the policy, the
`odom` and goal checks, inference and clipping. A new agent subclasses it with `observe()` and `act()`.

## Running

Install `onnxruntime` from pip, system-wide (rosdep key `python3-onnxruntime-pip`):

```bash
pip install --break-system-packages onnxruntime
```

With the sim or the boat's bringup running:

```bash
ros2 launch kingfisher_rl passthrough_thrusters.launch.py            # use_sim_time:=true in sim
ros2 topic pub --once /kingfisher/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: kingfisher/base_link}, pose: {position: {x: 8.0, y: 4.0}}}"
```

Then press **A** on the joystick: `drive_mux` hands `cmd_drive` to `control/cmd_drive`. **B** takes it
back. The launch starts `goal_publisher`, the agent, and `detection_goal` with `detections:=true`.
`policy:=<path>` picks another model.

## Models

`models/passthrough_thrusters/seed44.onnx` is the tracked sample: the Isaac Lab `kingfisher_direct`
policy `final-seed44` (dt 0.02 s, decimation 5, so 10 Hz). Other `.onnx` files under `models/` are
git-ignored but installed if present. `policy` is a path under `share/kingfisher_rl/models/`, or an
absolute path.

`scripts/benchmark_onnx.py -m <model>` times inference (not installed).

## Parameters

`config/passthrough_thrusters.yaml`; the goal parameters are in `kingfisher_goals`.

| parameter | default | |
|---|---|---|
| `policy` | `passthrough_thrusters/seed44.onnx` | |
| `odom_timeout` | 1.0 | s, a goal with older `odom` is skipped |
| `episode_gap` | 0.5 | s without goals before the previous action resets to 0 |
