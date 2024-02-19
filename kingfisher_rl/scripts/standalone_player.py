from gym import spaces
import numpy as np
import torch
import yaml
import rospkg
from rl_games.algos_torch.players import BasicPpoPlayerContinuous

config_folder= rospkg.RosPack().get_path('kingfisher_rl') + '/config/'
config_name = config_folder+'test.yaml'
policy_path = config_folder+ 'test.pth'

with open(config_name, 'r') as stream:
    cfg = yaml.safe_load(stream)

device='cuda'
num_obs = 10 
max_actions = 2
task_label = torch.ones((1), device=device, dtype=torch.float32)
observation_space = spaces.Dict({"state":spaces.Box(np.ones(num_obs) * -np.Inf, np.ones(num_obs) * np.Inf),
                                              "transforms":spaces.Box(low=-1, high=1, shape=(max_actions, 5)),
                                              "masks":spaces.Box(low=0, high=1, shape=(max_actions,))})

action_space = spaces.Box(low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32)


player = BasicPpoPlayerContinuous(cfg, observation_space, action_space, clip_actions=True, deterministic=True, device=device)
player.restore(policy_path)

robot_pos = [0.0, 0.0]
heading = [0.0, 0.0]
lin_vel = [0.0, 0.0]
ang_vel = [0.0]
target_positions = torch.tensor([[0.0, 0.0]], device=device, dtype=torch.float32)

current_state = {"position":robot_pos, "orientation": heading, "linear_velocity": lin_vel, "angular_velocity":ang_vel}

obs_buffer = torch.zeros((1, num_obs), device=device, dtype=torch.float32)
obs_buffer[:, 0:2] = torch.tensor(current_state["orientation"], device=device)
obs_buffer[:, 2:4] = torch.tensor(current_state["linear_velocity"], device=device)
obs_buffer[:, 4] = torch.tensor(current_state["angular_velocity"], device=device)
obs_buffer[:, 5] = task_label
obs_buffer[:, 6:8] = target_positions - torch.tensor(current_state["position"], device=device)
obs_buffer[:, 8:] = target_positions - torch.tensor(current_state["position"], device=device)
  

for i in range(1):
    # obs = dict({'obs':torch.tensor([1.0,0.5], dtype=torch.float32, device='cpu')})
    obs = dict({"state":obs_buffer,
                    "transforms":torch.zeros((1, 2, 5), device=device, dtype=torch.float32),
                    "masks":torch.zeros((1, 2), device=device, dtype=torch.long)})
    action = player.get_action(obs, is_deterministic=True)
    print("itteration: ", i)
    print("obs: ",obs)
    print("actions: ",action)
    print("")

