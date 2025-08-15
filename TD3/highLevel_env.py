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

class HighLevelEnv(gym.Env):
    def __init__(self, gazebo_sim_instance, low_level_policy_path):
        super().__init__()
        self.gazebo = gazebo_sim_instance
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32) # (x, y, goal_x, goal_y)
        self.action_space = spaces.Box(low=-5, high=5, shape=(2,), dtype=np.float32)

        self.map_info = None
        self.last_odom = None
        
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.odom = rospy.Subscriber(
            "/r1/odom", Odometry, self.odom_callback, queue_size=1
        )

        self.robot = Agent()


    def map_callback(self, msg):
        """
        Occupancy Map 的回调函数，用于更新 MapInfo。
        """
        # 从 OccupancyGrid 消息中提取数据
        map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        map_origin_x = msg.info.origin.position.x
        map_origin_y = msg.info.origin.position.y

        cell_size = msg.info.resolution
        # print(map_origin_x, map_origin_y, cell_size)
        # 初始化或更新 MapInfo 对象
        self.map_info = MapInfo(map_data, map_origin_x, map_origin_y, cell_size)

    def odom_callback(self, od_data):
        self.last_odom = od_data


    def reset(self, seed=None, options=None):

        return 

    def step(self, action): 
        
        self.robot.update_planning_state(self.map_info, location)
            
        return observation, reward, terminated, False, {}

