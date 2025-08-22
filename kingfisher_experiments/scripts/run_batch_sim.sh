name="opt"
exp="large_experiments.yaml"
exp_name="ideal"
base_dir="/home/luis/workspaces/bags_ws/tfr_exp/sim_bags"

onnx_path="rsl_rl/kingfisher_direct/2025-08-20_15-58-58_final-seed42/exported/policy.onnx"
onnx_path="rsl_rl/kingfisher_direct/2025-08-21_14-23-40_nodelay-seed42/exported/policy.onnx"

# Default values

mass="35.0"
cog_y="0.0"
mi_z="8.061"
rdl="5.83"
rdq="17.34"
sdl="0.0"
sdq="17.26"

policy="good.pth"

default_values() {
    mass="35.0"
    cog_y="0.0"
    mi_z="8.061"
    rdl="5.83"
    rdq="17.34"
    sdl="0.0"
    sdq="17.26"
}

start_gazebo() {
    roslaunch heron_gazebo heron_world.launch world_name:="empty" gui:=false &
    GAZEBO_PID=$!
    sleep 15
    echo "Gazebo started."
    sleep 1
}

kill_gazebo() {
    kill $GAZEBO_PID
    wait $GAZEBO_PID
    sleep 1
    echo "Gazebo terminated."
    sleep 1
}

start_rviz() {
    rviz -d ~/.rviz/kingfisher-sim.rviz &
    RVIZ_PID=$!
    sleep 5
    echo "Rviz started."
    sleep 1
}

kill_rviz() {
    kill $RVIZ_PID
    wait $RVIZ_PID
    sleep 1
    echo "Rviz terminated."
    sleep 1
}

start_environment_nodes() {
    roslaunch kingfisher_experiments experiment_environment_sim.launch max_goal_distance:=10.0 &
    ENVNODES_PID=$!
    sleep 10
    echo "Environment nodes started."
    sleep 1
}

kill_environment_nodes() {
    kill $ENVNODES_PID
    wait $ENVNODES_PID
    sleep 1
    echo "Environment nodes terminated."
    sleep 1
}

wait_experiment_to_finish() {
    while rosnode list | grep -q "/experiment_runner"
    do
        echo "Monitoring /experiment_runner..."
        sleep 5
    done
    sleep 1
}


start_rl() {
    roslaunch kingfisher_rl capture_agent_onnx.launch onnx_path:=$onnx_path &
    AGENT_PID=$!
    sleep 10
    echo "RL started."
    sleep 1
}

kill_agent() {
    kill $AGENT_PID
    wait $AGENT_PID
    sleep 1
    echo "Agent terminated."
    sleep 1
}

launch_heron() {
    roslaunch heron_gazebo spawn_heron.launch mass:=$mass cog_y:=$cog_y mi_z:=$mi_z rdl:=$rdl rdq:=$rdq sdl:=$sdl sdq:=$sdq &
    HERON_LAUNCH_PID=$!
    sleep 5
    echo "Heron launched"
    sleep 1
}

kill_heron() {
    kill $HERON_LAUNCH_PID
    wait $HERON_LAUNCH_PID
    sleep 1
    echo "Heron launch file terminated."
    rosservice call /gazebo/delete_model heron
    rosservice call /gazebo/reset_world
    sleep 2
    echo "Heron deleted."
    sleep 1
}

start_environment() {
    start_gazebo
    start_rviz
    start_environment_nodes
}

kill_environment() {
    kill_environment_nodes
    kill_rviz
    kill_gazebo
}

run_experiments() {
    # This is expected to block until the experiment is finished
    roslaunch kingfisher_experiments experiment_runner.launch non_stop:=true &
    echo "Experiment started."
    EXPERIMENT_PID=$!
    wait $EXPERIMENT_PID
    echo "Experiment finished."
}

start_bag_recorder() {
    roslaunch kingfisher_experiments bag_recorder.launch base_dir:=$base_dir exp_name:=$exp_name &
    BAG_RECORDER_PID=$!
    sleep 2
    echo "Bag recorder started."
    sleep 2
}

stop_bag_recorder() {
    kill $BAG_RECORDER_PID
    wait $BAG_RECORDER_PID
    sleep 1
    echo "Bag recorder stopped."
    sleep 1
}

test_rl() {
    start_rl $1 $2
    start_bag_recorder
    run_experiments
    stop_bag_recorder
    kill_agent
}


# TODO: 
# Insert the perception nodes
# simulated_waste
# fake_detector (with parameters)
# SWITCH BETWEEN each one (when using perception noise or not)
# rosservice call /goal_mux/select /move_base_simple/goal
# rosservice call /goal_mux/select /waste_detector/goal


# name="_optimal"

# exp="small_experiments.yaml"
# exp="tiny_experiments.yaml"
exp="experiments.yaml"
start_environment


name="_default"
default_values
launch_heron
test_rl high high_dr_last.pth

# test_mpc
kill_heron

# name="_cog000"
# default_values
# cog_y="0.00"
# launch_heron
# test_rl high high_dr_last.pth
# test_mpc
# kill_heron



kill_environment


# exp="small_experiments.yaml"

# cog_ys=(0.0 0.01 0.05)
# for cog_y in "${cog_ys[@]}"; do
# done


# for mass in "${masses[@]}"; do
#         for mi_z in "${mi_zs[@]}"; do
#             for rdl in "${rdls[@]}"; do
#                 for sdl in "${sdls[@]}"; do
#                     name="_mas_${mass}_cog_${cog_y}_miz_${mi_z}_rdl_${rdl}_sdl_${sdl}"
#                     launch_heron
#                     test_both
#                     kill_heron
#                 done
#             done
#         done
#     done

# default_values
# name="_opt_tiny"
# launch_heron
# test_both
# kill_heron

# default_values
# cog_y="0.1"
# name="_cog_tiny"

# launch_heron
# test_both
# kill_heron
