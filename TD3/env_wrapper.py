# env_wrapper.py
import gymnasium as gym
from gymnasium import spaces
import numpy as np

from velodyne_env import GazeboEnv

class GazeboWrapper(gym.Wrapper):
    def __init__(self, env):
        super().__init__(env)
        
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)

        environment_dim = 20 
        robot_dim = 4
        state_dim = environment_dim + robot_dim
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(state_dim,), dtype=np.float32)

        print("GazeboWrapper 已成功包装原始环境！")

    def reset(self, **kwargs):

        state = self.env.reset()
        
        observation = np.array(state, dtype=np.float32)
        info = {}
        return observation, info

    def step(self, action):

        linear_vel = (action[0] + 1) / 2
        angular_vel = action[1]
        a_in = [linear_vel, angular_vel]
        
        next_state, reward, done, target = self.env.step(a_in)
        
        observation = np.array(next_state, dtype=np.float32)
        reward = float(reward)
        terminated = bool(done) 
        truncated = False      
        info = {'target': target} 
        
        return observation, reward, terminated, truncated, info
