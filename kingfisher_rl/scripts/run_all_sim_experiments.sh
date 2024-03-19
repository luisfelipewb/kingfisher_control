#!/bin/bash

# CONFIGURATION VARIABLES
models=(
Capture_Nominal-DR0
Capture_Nominal-DR50
Capture_SysID-DR0
Capture_SysID-DR50
)

WEIGHTS_BASE_PATH="/home/lwolfbat/uuv_ws/src/kingfisher_control/kingfisher_rl/output"
BAGS_BASE_PATH="/home/lwolfbat/uuv_ws/src/kingfisher_control/kingfisher_rl/output"
BAG_TOPICS="/pose_gt /cmd_vel /cmd_drive /rl_status /move_base_simple/goal"
DEBUG=false #only true of false

test_model_paths () {
      for model in "${models[@]}"; do
        if [ ! -f "$WEIGHTS_BASE_PATH/$model.pth" ]; then
          echo "Model $model does not exist at $WEIGHTS_BASE_PATH/$model.pth"
          exit 1
        fi
        ls -l $WEIGHTS_BASE_PATH/$model.pth
        # cp $WEIGHTS_BASE_PATH/$model/nn/$model.pth /home/lwolfbat/uuv_ws/src/kingfisher_control/kingfisher_rl/

        # Parse task and model name
        task_name=${model%_*}
        model_name=${model#*_}
        echo "Task:$task_name"
        echo "Model:$model_name"
      done
      # exit 0
}

wait_for_gazebo() {
  # Loop indefinitely until the topic "/pose_gt" is found which means that Gazebo is ready
  while true; do
    if rostopic list | grep -q "/pose_gt"; then
      echo "Topic /pose_gt is being published."
      sleep 3
      break
    else
      echo "Waiting for gazebo..."
      sleep 5
    fi
  done
}

use_gazebo_params() {
    base_dir="/home/lwolfbat/uuv_ws/src/heron/heron_description/urdf/"
    param_type=$1
    # check if the param type is valid, should be either "original" or "sysid"
    if [ "$param_type" != "Orig" ] && [ "$param_type" != "SysID" ]; then
        echo "Invalid gazebo param type: $param_type"
        exit 1
    fi
    
    # Change snippets file
    snippets_path=" ${base_dir}/snippets.xacro"
    snippets_target="${base_dir}/snippets_${param_type}.xacro"
    ln -sf $snippets_target $snippets_path

    
    echo "Using $param_type gazebo params"
}

run_gazebo_sim_tests() {

    sim_parameters=$1
    # check if the param type is valid, should be either "original" or "sysid"
    if [ "$sim_parameters" != "Orig" ] && [ "$sim_parameters" != "SysID" ]; then
        echo "Invalid gazebo simulation parameters: $sim_parameters"
        exit 1
    fi

    sim_name="Sim$sim_parameters"

    use_gazebo_params $sim_parameters


    # Setup Gazebo simulation environment
    echo "Starting Gazebo simulation"
    roslaunch heron_gazebo heron_world.launch gui:=$DEBUG &
    GAZEBO_PID=$!
    wait_for_gazebo
    echo "Gazebo simulation started with PID: $GAZEBO_PID"

    # Start RViz DEBUG is set to true
    if [ "$DEBUG" = true ]; then
        echo "Starting RViz"
        rviz -d /home/lwolfbat/.rviz/kingfisher_rl.rviz &
        RVIZ_PID=$!
        echo "RViz started with PID: $RVIZ_PID"
    fi
    

    for model in "${models[@]}"; do 
        echo "Running Simulation:$sim_name Model:$model"
        model_path="$WEIGHTS_BASE_PATH/$model/nn/$model.pth"
        echo "Model path: $model_path"
        
        # Parse task and model name
        task_name=${model%_*}
        model_name=${model#*_}

        # Fix the bag name
        bag_name=${task_name}_${sim_name}_${model_name}.bag
        bag_path="$BAGS_BASE_PATH/$bag_name"

        rosbag record -O $bag_path $BAG_TOPICS  __name:=recorder &
        BAG_RECORD_PID=$!
        sleep 1
        echo "Recording bag name: $bag_name"

        # Run the agent
        roslaunch kingfisher_rl capture_sim.launch policy:=$model_path &
        AGENT_PID=$!
        sleep 1
        echo "Running the agent with model: $model_path"

        # Run evalution script with multple goals
        roslaunch kingfisher_rl test_capture_sim.launch bag_name:=$bag_path 

        echo "Killing the angnet and bag recording"
        rosnode kill /goto_rl
        rosnode kill /recorder
        wait $AGENT_PID
        wait $BAG_RECORD_PID
        echo "Killed the agent and bag recording"

        sleep 1
        echo ""
    done

    sleep 1

    echo "Killing Gazebo"
    kill $GAZEBO_PID
    wait $GAZEBO_PID
    echo "Killed Gazebo"

    if [ "$DEBUG" = true ]; then
        echo "Killing RViz"
        kill $RVIZ_PID
        wait $RVIZ_PID
        echo "Killed Gazbo and Rviz"
    fi
}

test_model_paths

echo "Running all simulation experiments"


# run_gazebo_sim_tests "Orig"
run_gazebo_sim_tests "SysID"


echo "All simulation experiments finished"

rosrun kingfisher_rl sim_metrics.py

echo "Finished... Good luck with your analysis!"
exit 0
