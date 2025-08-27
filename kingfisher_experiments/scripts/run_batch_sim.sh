
exp_name="thename"
base_dir="/home/luis/workspaces/bags_ws/tfr_exp/sim_bags"

onnx_path="rsl_rl/kingfisher_direct/final-seed42/exported/policy.onnx"

exp_file="experiments.yaml"
dst_thres=0.3

localization_delay=0
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
    roslaunch kingfisher_experiments experiment_environment.launch \
        max_goal_distance:=10.0 \
        odom_topic:=/pose_gt \
        loe_right:=$loe_right \
        pixel_noise_radius:=$pixel_noise_radius \
        distance_threshold:=$dst_thres \
        localization_delay:=$localization_delay &

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
    roslaunch kingfisher_experiments experiment_runner.launch \
        non_stop:=true \
        exp_file:=$exp_file \
        distance_threshold:=$dst_thres \
        exp_name:=$exp_name \
        stable_start_time:=3.0 \
        vel_threshold:=0.05 \
        &
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
    start_rl
    start_bag_recorder
    run_experiments
    stop_bag_recorder
    kill_agent
}

enable_perception_noise() {
    rosservice call /goal_mux/select /waste_detector/goal
}

disable_perception_noise() {
    rosservice call /goal_mux/select /move_base_simple/goal
}


default_values
launch_heron
start_environment
enable_perception_noise
test_rl
kill_heron
kill_environment



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
