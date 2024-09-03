Steps for running the experiment. 

Make sure the sbg is working

Start the joy node in the field computer

Start the environmet for the experiment (mux, odom converter, velocity tracker..)
roslaunch kingfisher_experiments experiment_environment.launch



Start the experiment with bag recorder
roslaunch kingfisher_experiments experiment_runner.launch base_dir:="/home/kingfisher/bags/" mpc:=true exp_name:=mpc
roslaunch kingfisher_experiments experiment_runner.launch base_dir:="/home/kingfisher/bags/" rl:=true exp_name:=rldf
roslaunch kingfisher_experiments experiment_runner.launch base_dir:="/home/kingfisher/bags/" rl:=true exp_name:=rlgood policy:=good.pth
roslaunch kingfisher_experiments experiment_runner.launch base_dir:="/home/kingfisher/bags/" rl:=true exp_name:=rlgood policy:=bad.pth

roslaunch kingfisher_experiments experiment_runner.launch base_dir:="/home/kingfisher/bags/" mpc:=true exp_name:=mpc2
roslaunch kingfisher_experiments experiment_runner.launch base_dir:="/home/kingfisher/bags/" rl:=true exp_name:=rldf2