# kingfisher_sysid

System identification: open-loop thrust sequences and the boat's response, logged to CSV. Renamed
from the ROS 1 `kingfisher_sid`.

## Running

`sequence_runner` plays one experiment from `config/experiments.yaml` on `control/cmd_drive`, then
writes `<output_dir>/<experiment>.csv` and exits. Give it `cmd_drive` with joystick **A**
(`drive_mux`), like the agents. **B** takes it back at any time.

```bash
ros2 run kingfisher_sysid sequence_runner --ros-args -r __ns:=/kingfisher \
  -p experiment:=acceleration_1.0 -p output_dir:=~/sysid      # -p use_sim_time:=true in sim
```

An experiment is a list of sections, each a thrust pair held for a time:

```yaml
- experiment_name: acceleration_1.0
  sections:
  - {duration: 5.0, tl: 0.0, tr: 0.0}
  - {duration: 10.0, tl: 1.0, tr: 1.0}
```

## Interface

| | name | type | |
|---|---|---|---|
| sub | `odom` | `nav_msgs/Odometry` | twist in the body frame |
| pub | `control/cmd_drive` | `kingfisher_msgs/Drive` | at `rate` |
| pub | `experiment_name` | `std_msgs/String` | at `rate`, for bags |

The CSV has one row per command: `time` (s since the start), `thr_r`, `thr_l`, `pos_x`, `pos_y`,
`lin_x`, `lin_y`, `ang_z` (body frame, from `odom`), `roll`, `pitch`, `yaw` (rad).

## Parameters

| parameter | default | |
|---|---|---|
| `experiment` | `acceleration_1.0` | an `experiment_name` |
| `experiments_file` | `share/kingfisher_sysid/config/experiments.yaml` | |
| `output_dir` | `.` | |
| `rate` | 10.0 | Hz |

## Scripts

`scripts/` holds the offline tools, still ROS 1 and not installed: `generate_experiments.py`, the
`plot_*` scripts, `remove_initial_offset.py`, and `experiment_runner.py` (runs every experiment with
`roslaunch`).
