name="opt"
exp="large_experiments.yaml"
# ./run_sim_mpc.sh $name $exp
# ./run_sim_rl.sh $name $exp



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
    rviz -d ~/.rviz/kingfisher_rl.rviz &
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
    roslaunch kingfisher_experiments experiment_environment_sim.launch &
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

start_mpc() {
    rosservice call /gazebo/reset_world
    roslaunch kingfisher_experiments experiment_runner_sim.launch mpc:=true exp_name:="mpc$name" base_dir:="/home/luis/workspaces/bags/sim" exp_file:=$exp &
    AGENT_PID=$!
    sleep 10
    echo "MPC started."
    sleep 1
}

start_rl() {
    rosservice call /gazebo/reset_world
    roslaunch kingfisher_experiments experiment_runner_sim.launch rl:=true exp_name:="rl$1$name" base_dir:="/home/luis/workspaces/bags/sim" exp_file:=$exp policy:=$2 &
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

test_rl() {
    start_rl $1 $2
    wait_experiment_to_finish
    kill_agent
}

test_mpc() {
    start_mpc
    wait_experiment_to_finish
    kill_agent
}

test_both() {
    test_rl
    test_mpc
}

# Minimum sequence
# default_values
# start_environment
# launch_heron
# test_both
# wait_experiment_to_finish
# kill_heron
# kill_environment

# name="_optimal"


# exp="small_experiments.yaml"
# exp="tiny_experiments.yaml"
exp="final_large_0.yaml"
exp="small_experiments.yaml"
exp="small_experiments.yaml"
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

# name="_cog025"
# default_values
# cog_y="0.025"
# launch_heron
# test_rl high high_dr_last.pth
# test_mpc
# kill_heron

# name="_cog050"
# default_values
# cog_y="0.050"
# launch_heron
# test_rl high high_dr_last.pth
# test_mpc
# kill_heron

# name="_cog075"
# default_values
# cog_y="0.075"
# launch_heron
# test_rl high high_dr_last.pth
# test_mpc
# kill_heron

# name="_cog100"
# default_values
# cog_y="0.10"
# launch_heron
# test_rl high high_dr_last.pth
# test_mpc
# kill_heron

# name="_cog125"
# default_values
# cog_y="0.125"
# launch_heron
# test_rl high high_dr_last.pth
# test_mpc
# kill_heron

# name="_cog150"
# default_values
# cog_y="0.150"
# launch_heron
# test_rl high high_dr_last.pth
# test_mpc
# kill_heron



# name="_rd0"
# default_values
# rdl="0.0"
# launch_heron
# test_rl high high_dr_last.pth
# test_mpc
# kill_heron

# name="_rd2"
# default_values
# rdl="2.0"
# launch_heron
# test_rl high high_dr_last.pth
# test_mpc
# kill_heron

# name="_rd4"
# default_values
# rdl="4.0"
# launch_heron
# test_rl high high_dr_last.pth
# test_mpc
# kill_heron

# name="_rd5"
# default_values
# rdl="5.0"
# launch_heron
# test_rl high high_dr_last.pth
# test_mpc
# kill_heron

# name="_rd6"
# default_values
# rdl="6.0"
# launch_heron
# test_rl high high_dr_last.pth
# test_mpc
# kill_heron

# name="_rd8"
# default_values
# rdl="8.0"
# launch_heron
# test_rl high high_dr_last.pth
# test_mpc
# kill_heron

# name="_rd10"
# default_values
# rdl="10.0"
# launch_heron
# test_rl high high_dr_last.pth
# test_mpc
# kill_heron

# name="_rd12"
# default_values
# rdl="12.0"
# launch_heron
# test_rl high high_dr_last.pth
# test_mpc
# kill_heron

# name="_rd15"
# default_values
# rdl="15.0"
# launch_heron
# test_rl high high_dr_last.pth
# test_mpc
# kill_heron

# name="_rd20"
# default_values
# rdl="20.0"
# launch_heron
# test_rl high high_dr_last.pth
# test_mpc
# kill_heron


# name="_low_mild"
# default_values  
# mass="38.0"
# cog_y="0.02"
# mi_z="10"
# rdl="8.0"
# sdl="2.0"
# launch_heron
# # test_both
# test_rl
# kill_heron

# name="_low_heavy"
# default_values  
# mass="40.0"
# cog_y="0.06"
# mi_z="12"
# rdl="10.0"
# sdl="4.0"
# launch_heron
# # test_both
# test_rl
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

# roslaunch heron_gazebo spawn_heron.launch mass:=35.0 cog_y:=0.01 mi_z:=12.0 rdl:=0.0 rdq:=2.0 sdl:=2 sdq:=2
