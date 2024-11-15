Steps for running the experiment. 

# Make sure the sbg is working
roslaunch sbg_driver sbg_device.launch
roslaunch sbg_localization_manager sbg_localization_processor.launch

# Start the joy node in the field computer

# Start the environmet for the experiment (mux, odom converter, velocity tracker..)
roslaunch kingfisher_experiments experiment_environment.launch

# Record a bag with no actions for measuring weather disturbance.
roslaunch kingfisher_experiments experiment_runner.launch base_dir:="/home/kingfisher/bags/" mpc:=true exp_name:=weather

# none
roslaunch kingfisher_experiments experiment_runner.launch base_dir:="/home/kingfisher/bags/" rl:=true exp_name:=rlrd5_none policy:=rd5s42.pth
roslaunch kingfisher_experiments experiment_runner.launch base_dir:="/home/kingfisher/bags/" rl:=true exp_name:=rlrd1_none policy:=rd1s42.pth
roslaunch kingfisher_experiments experiment_runner.launch base_dir:="/home/kingfisher/bags/" rl:=true exp_name:=noreturn_none policy:=rd1finish.pth

roslaunch kingfisher_experiments experiment_runner.launch base_dir:="/home/kingfisher/bags/" mpc:=true exp_name:=mpcrd5_none
roslaunch kingfisher_experiments experiment_runner.launch base_dir:="/home/kingfisher/bags/" mpc:=true exp_name:=mpcrd20_none

roslaunch kingfisher_experiments experiment_runner.launch base_dir:="/home/kingfisher/bags/" rl:=true exp_name:=finish_none policy:=rd1noreturn.pth
roslaunch kingfisher_experiments experiment_runner.launch base_dir:="/home/kingfisher/bags/" rl:=true exp_name:=rlrd20_none policy:=rd20s42.pth


# disturbances right
roslaunch kingfisher_experiments experiment_runner.launch base_dir:="/home/kingfisher/bags/" rl:=true exp_name:=rlrd5_right policy:=rd5s42.pth
roslaunch kingfisher_experiments experiment_runner.launch base_dir:="/home/kingfisher/bags/" rl:=true exp_name:=rlrd1_right policy:=rd1s42.pth
roslaunch kingfisher_experiments experiment_runner.launch base_dir:="/home/kingfisher/bags/" rl:=true exp_name:=noreturn_right policy:=rd1finish.pth


roslaunch kingfisher_experiments experiment_runner.launch base_dir:="/home/kingfisher/bags/" mpc:=true exp_name:=mpcrd5_right
roslaunch kingfisher_experiments experiment_runner.launch base_dir:="/home/kingfisher/bags/" mpc:=true exp_name:=mpcrd20_right

roslaunch kingfisher_experiments experiment_runner.launch base_dir:="/home/kingfisher/bags/" rl:=true exp_name:=finish_right policy:=rd1noreturn.pth
roslaunch kingfisher_experiments experiment_runner.launch base_dir:="/home/kingfisher/bags/" rl:=true exp_name:=rlrd20_right policy:=rd20s42.pth