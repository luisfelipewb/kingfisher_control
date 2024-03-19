# kingfisher_control

This repository contains two ROS packages for controlling the Clearpath Kingfisher Unmanned Surface Vehicle (USV).

## Packages

### 1. kingfisher_sid

This package contains scripts for performing automated system identification with the Clearpath Kingfisher USV.

#### Usage

To perform system identification, you can use the following launch file:

```bash
roslaunch kingfisher_sid run_sequence.launch [experiment_name]
```

### 2. kingfisher_rl

This package contains ROS wrappers to execute capture tasks with the Clearpath Kingfisher USV.

#### Usage

To execute capture agent in simulation, you can use the following command. It will start and agent that subscribes to to a goal message.

```bash
roslaunch kingfisher_rl capture_sim.launch policy:=weights.pth
```

For automated tests the following command can be handy.
```bash
roslaunch kingfisher_rl test_capture_sim.launch
```
## Dependencies

- https://github.com/heron/heron_simulator

Make sure to have dependencies installed before using the packages in this repository.


