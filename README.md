# kingfisher_control

Control for the Kingfisher ASV, ROS 2 Jazzy: everything that produces `cmd_drive`. The platform
(`kingfisher`) consumes it. The ROS 1 code is on the `noetic` branch.

| package | what it does |
|---|---|
| `kingfisher_twist` | `cmd_vel` → `teleop/cmd_drive`, for arcade teleop |
| `kingfisher_goals` | `goal_publisher`: a goal in any frame → a body-frame goal stream (`control/goal`) that ends when the goal is reached or times out; `detection_goal`: goals from detections; `waypoint_goal`: waypoint following, with detours to detections; `virtual_detections`: virtual bottles, in sim or on the water |
| `kingfisher_rl` | RL agents (ONNX): `passthrough_thrusters`, one command per goal on `control/cmd_drive` |
| `kingfisher_sysid` | `sequence_runner`: open-loop thrust sequences on `control/cmd_drive`, logged to CSV |

Controllers publish on `control/cmd_drive`. The bringup's and the sim's `drive_mux` passes either
`teleop/cmd_drive` (the default) or `control/cmd_drive` to `cmd_drive`. Press **A** on the joystick to
hand over to the controller and **B** to take back control. When a controller stops publishing, the
MCU (or `cmd_drive_translate` in sim) zeroes the thrusters.

```bash
ros2 launch kingfisher_rl passthrough_thrusters.launch.py      # use_sim_time:=true in sim
ros2 launch kingfisher_goals waypoints.launch.py scenario:=grid_small virtual_detections:=true
```

The ROS 1 `kingfisher_experiments` (experiment runner, velocity PID, batch scripts) is on `noetic`.

Each package's README has its interface and parameters.
