import gymnasium as gym
from gymnasium import spaces
import numpy as np
from stable_baselines3 import SAC
# from gazebo_sim import GazeboSim
from agent import Agent
from utils import *
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from copy import deepcopy
from parameter import *
from highLevel_env import HighLevelEnv
import time
from collections import deque
import random
import torch
import torch.nn as nn
import torch.optim as optim
from model import PolicyNet, QNet
from parameter import *
class ReplayBuffer:
    def __init__(self, buffer_size):
        self.buffer = deque(maxlen=buffer_size)

    def push(self, state, action, reward, next_state, done):
        """将一次交互的数据存入缓冲区"""
        self.buffer.append((state, action, reward, next_state, done))

    def sample(self, batch_size):
        """从缓冲区中随机采样一个批次的数据"""
        # 从 deque 中随机采样
        batch = random.sample(self.buffer, batch_size)
        
        # 将数据解压并分别打包
        states, actions, rewards, next_states, dones = zip(*batch)

        # 对字典式的 state 和 next_state 进行特殊处理
        # 将 list of dicts 转换成 dict of lists (然后转换成 tensor)
        states = {key: torch.cat([s[key] for s in states], dim=0) for key in states[0]}
        next_states = {key: torch.cat([s[key] for s in next_states], dim=0) for key in next_states[0]}

        # 将其他数据转换为 tensor
        actions = torch.tensor(np.array(actions), dtype=torch.int64).unsqueeze(1)
        rewards = torch.tensor(np.array(rewards), dtype=torch.float32).unsqueeze(1)
        dones = torch.tensor(np.array(dones), dtype=torch.float32).unsqueeze(1)

        return states, actions, rewards, next_states, dones

    def __len__(self):
        return len(self.buffer)
    
class SAC_Agent:
    def __init__(self, env, policy_net_class, q_net_class, lr=3e-4, gamma=0.99, 
                 tau=0.005, buffer_size=1000000, alpha=0.2):
        
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"Using device: {self.device}")

        self.env = env
        self.gamma = gamma
        self.tau = tau
        self.alpha = alpha # 在离散 SAC 中，alpha 通常是固定的超参数

        # --- 初始化网络 ---
        # Actor (策略网络)
        self.actor = policy_net_class(NODE_INPUT_DIM, 128).to(self.device)
        
        # Critic (价值网络), SAC 需要两个 Q 网络来减少过高估计
        self.critic1 = q_net_class(NODE_INPUT_DIM, 128).to(self.device)
        self.critic2 = q_net_class(NODE_INPUT_DIM, 128).to(self.device)
        
        # Target Critic Networks (目标价值网络)
        self.target_critic1 = q_net_class(NODE_INPUT_DIM, 128).to(self.device)
        self.target_critic2 = q_net_class(NODE_INPUT_DIM, 128).to(self.device)

        # 复制参数到目标网络
        self.target_critic1.load_state_dict(self.critic1.state_dict())
        self.target_critic2.load_state_dict(self.critic2.state_dict())
        
        # --- 初始化优化器 ---
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=lr)
        self.critic1_optimizer = optim.Adam(self.critic1.parameters(), lr=lr)
        self.critic2_optimizer = optim.Adam(self.critic2.parameters(), lr=lr)

        # --- 初始化经验回放缓冲区 ---
        self.replay_buffer = ReplayBuffer(buffer_size)

    def select_action(self, state):
        """根据当前状态选择一个动作"""
        # 将 numpy 观测转换为 torch tensor 并增加一个批次维度
        state_tensor = {key: torch.tensor(val).unsqueeze(0).to(self.device) for key, val in state.items()}
        
        with torch.no_grad():
            # 从 actor 获取动作的对数概率
            action_log_probs = self.actor(**state_tensor)
            action_probs = torch.exp(action_log_probs)
            
            # 根据概率分布进行采样
            action = torch.multinomial(action_probs, 1).item()
            
        return action

    def update(self, batch_size):
        """执行一次网络更新"""
        if len(self.replay_buffer) < batch_size:
            return # 缓冲区数据不足，不更新

        # 1. 从缓冲区采样
        states, actions, rewards, next_states, dones = self.replay_buffer.sample(batch_size)
        
        # 将所有数据移动到正确的设备
        actions = actions.to(self.device)
        rewards = rewards.to(self.device)
        dones = dones.to(self.device)
        for key in states:
            states[key] = states[key].to(self.device)
            next_states[key] = next_states[key].to(self.device)

        # --- 更新 Critic 网络 ---
        with torch.no_grad():
            # a. 计算下一个状态的动作及其对数概率
            next_action_log_probs = self.actor(**next_states)
            next_action_probs = torch.exp(next_action_log_probs)
            
            # b. 计算下一个状态的 Q 值
            q1_next_target = self.target_critic1(**next_states)
            q2_next_target = self.target_critic2(**next_states)
            q_next_target = torch.min(q1_next_target, q2_next_target)

            # c. 计算目标 Q 值 (离散 SAC 的公式)
            # V(s') = sum( pi(a'|s') * (Q_target(s',a') - alpha * log(pi(a'|s'))) )
            next_value = torch.sum(next_action_probs * (q_next_target - self.alpha * next_action_log_probs), dim=-1, keepdim=True)
            target_q_value = rewards + (1 - dones) * self.gamma * next_value

        # d. 计算 Q loss
        q1 = self.critic1(**states).gather(1, actions.long())
        q2 = self.critic2(**states).gather(1, actions.long())
        
        critic1_loss = nn.MSELoss()(q1, target_q_value)
        critic2_loss = nn.MSELoss()(q2, target_q_value)

        # e. 更新 critic 网络
        self.critic1_optimizer.zero_grad()
        critic1_loss.backward()
        self.critic1_optimizer.step()

        self.critic2_optimizer.zero_grad()
        critic2_loss.backward()
        self.critic2_optimizer.step()

        # --- 更新 Actor 网络 ---
        action_log_probs = self.actor(**states)
        action_probs = torch.exp(action_log_probs)
        
        q1_policy = self.critic1(**states)
        q2_policy = self.critic2(**states)
        q_policy = torch.min(q1_policy, q2_policy)
        
        # a. 计算 actor loss
        # actor_loss = sum( pi(a|s) * (alpha * log(pi(a|s)) - Q(s,a)) )
        actor_loss = (action_probs * (self.alpha * action_log_probs - q_policy)).sum(dim=-1).mean()
        
        # b. 更新 actor 网络
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()
        
        # --- 更新目标网络 (软更新) ---
        self._soft_update(self.target_critic1, self.critic1)
        self._soft_update(self.target_critic2, self.critic2)

    def _soft_update(self, target, source):
        for target_param, source_param in zip(target.parameters(), source.parameters()):
            target_param.data.copy_(self.tau * source_param.data + (1.0 - self.tau) * target_param.data)
            

if __name__ == "__main__":
    # --- 超参数 ---
    MAX_EPISODES = 1000
    MAX_STEPS_PER_EPISODE = 200
    BATCH_SIZE = 256
    UPDATE_EVERY = 50 # 每多少步更新一次网络
    LEARNING_STARTS = 1000 # 收集多少步后才开始更新

    env = HighLevelEnv()
    agent = SAC_Agent(env, PolicyNet, QNet)
    
    total_steps = 0
    print("Starting training...")
    
    for episode in range(MAX_EPISODES):
        
        observation, info = env.reset()
        episode_reward = 0
        
        for step in range(MAX_STEPS_PER_EPISODE):    
            action = [1, 0]
            next_observation, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            
            agent.replay_buffer.push(observation, action, reward, next_observation, done)
            
            observation = next_observation
            episode_reward += reward
            total_steps += 1
            
            # d. 如果收集到足够的数据，则更新网络
            if total_steps > LEARNING_STARTS and total_steps % UPDATE_EVERY == 0:
                agent.update(BATCH_SIZE)
            
            if done:
                break
        
        print(f"Episode: {episode+1}, Steps: {step+1}, Total Steps: {total_steps}, Reward: {episode_reward:.2f}")