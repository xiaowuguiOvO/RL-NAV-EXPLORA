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
from velodyne_env import GazeboEnv
import time

class HighLevelEnv(gym.Env):
    def __init__(self):
        super().__init__()
        
        self.observation_space = spaces.Dict({
            # SHAPE: (填充后的节点数, 特征维度)
            'node_inputs': spaces.Box(
                low=-np.inf, high=np.inf,
                shape=(NODE_PADDING_SIZE, NODE_INPUT_DIM),
                dtype=np.float32
            ),
            
            # SHAPE: (填充后的节点数,) -> 对应每个节点的掩码 (mask)
            'node_padding_mask': spaces.Box(
                low=0, high=1,
                shape=(NODE_PADDING_SIZE,),
                dtype=np.uint8 # 对0/1掩码建议使用uint8类型
            ),
            
            # SHAPE: (填充后的节点数, 填充后的节点数)
            'edge_mask': spaces.Box(
                low=0, high=1,
                shape=(NODE_PADDING_SIZE, NODE_PADDING_SIZE),
                dtype=np.float32
            ),
            
            # VALUE: 一个从 0 到 NODE_PADDING_SIZE-1 的整数
            'current_index': spaces.Discrete(NODE_PADDING_SIZE),

            # SHAPE: (填充后的邻居数,)
            # 注意：键名是 'next_edge' 以匹配你的函数返回值
            'next_edge': spaces.Box(
                low=0, high=NODE_PADDING_SIZE,
                shape=(K_SIZE,),
                dtype=np.float32 
            ),
            
            # SHAPE: (填充后的邻居数,) -> 对应每个邻居的掩码
            'edge_padding_mask': spaces.Box(
                low=0, high=1,
                shape=(K_SIZE,),
                dtype=np.uint8
            )
        })


        
        self.action_space = spaces.Discrete(K_SIZE) 
        self.reward_range = (-float('inf'), float('inf'))
        
        self.map_info = None
        self.last_odom = None
        
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.odom = rospy.Subscriber(
            "/r1/odom", Odometry, self.odom_callback, queue_size=1
        )

        self.robot = Agent(device='cpu', plot=False)

        self.env = GazeboEnv("test_simulator.launch", 20)
        print('init high level env done')

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
        # print(f"--- DEBUG: 收到新全局地图，尺寸: {map_data.shape} ---") 

    def odom_callback(self, od_data):
        self.last_odom = od_data


    def reset(self, seed=None, options=None):
        self.env.reset()
        self.robot.reset()
        robot_location = np.array([0, 0])
        while self.map_info is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for map data to be received...")
            rospy.sleep(0.5) # 等待0.5秒，避免CPU空转
        
        self.robot.update_planning_state(self.map_info, robot_location)
        self.robot.update_key_node_observation()
        observation = self.robot.get_pandding_observation()
        
        return observation, {}

    def calculate_reward(self):
        reward = 0
        return reward

    def step(self, action): 

        x = self.last_odom.pose.pose.position.x
        y = self.last_odom.pose.pose.position.y
        location = np.array([x, y])
        robot_node_location = location
        if self.robot.node_manager.nodes_dict.__len__() == 0:
            robot_node_location = [0, 0]
        else:
            nearest_node = self.robot.node_manager.nodes_dict.nearest_neighbors(location.tolist(), 1)[0]
            node_coords = nearest_node.data.coords
            robot_node_location = node_coords

        t1 = time.time()
        self.robot.update_planning_state(self.map_info, robot_node_location)
        t2 = time.time()
        # print("update planning state", t2 - t1)
        # key_node_coords, utility, guidepost, adjacent_matrix, neighbor_indices = self.robot.update_key_node_observation()
        # observation = self.robot.get_next_observation(robot_node_location)
        t1 = time.time()
        observation = self.robot.get_pandding_observation()
        t2 = time.time()
        # print("get observation time", t2 - t1)
        # reward
        reward = self.calculate_reward()
        
        # terminated
        terminated = False
        
        self.robot.publish_node_markers()
        self.robot.publish_frontier_markers()

        action = [0.1, 1.0]
        self.env.step(action)
        
        
        return observation, reward, terminated, False, {}


if __name__ == "__main__":
    # rospy.init_node("high_level_env", anonymous=True)
    env = HighLevelEnv()
    env = gym.wrappers.FlattenObservation(env)
    env.reward_range = env.env.reward_range
    
    model = SAC(
        policy="MlpPolicy",
        env=env,
        learning_rate=1e-4,
        buffer_size=1_000_000,
        verbose=1
    )
    
    print('start training')
    model.learn(total_timesteps=1000) # 先只训练1000步来测试
