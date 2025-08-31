from velodyne_env import GazeboEnv
from geometry_msgs.msg import Twist
import time
import rospy
from nav_msgs.msg import Odometry
from viewpoint_manager import ViewpointManager
import numpy as np
from agent import Agent
from nav_msgs.msg import OccupancyGrid
from utils import *
map_info = None

def map_callback(msg):
    """
    Occupancy Map 的回调函数，用于更新 MapInfo。
    """
    global map_info
    # 从 OccupancyGrid 消息中提取数据
    map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
    map_origin_x = msg.info.origin.position.x
    map_origin_y = msg.info.origin.position.y

    cell_size = msg.info.resolution
    # print(map_origin_x, map_origin_y, cell_size)
    # 初始化或更新 MapInfo 对象
    
    map_info = MapInfo(map_data, map_origin_x, map_origin_y, cell_size)
    
environment_dim = 20
env = GazeboEnv("test_simulator.launch", environment_dim)
rospy.Subscriber('/map', OccupancyGrid, map_callback)
viewpoint_manager = ViewpointManager()


try:
    rospy.wait_for_message('/r1/odom', Odometry, timeout=10)
    print("Odometry message received. Starting control loop.")
except rospy.ROSException:
    print("Did not receive odom message after 10 seconds. Exiting.")
    exit()

agent = Agent(policy_net=None, device='cpu', plot=False)

# vel_cmd = Twist()
# vel_cmd.linear.x = 0.5  # Set linear velocity
# vel_cmd.angular.z = 0.0  # Set angular velocity
while True:
    if map_info is None:
        # 说明还没收到 map，等一下
        continue  
    action_in = [0.5, 1]  # Example action input
    robot_location = env.get_robot_location()
    robot_location = np.array(robot_location)
    # viewpoint_manager.update(robot_location)
    robot_node_location = robot_location
    if agent.node_manager.nodes_dict.__len__() == 0:
        robot_node_location = [0, 0]
    else:
        nearest_node = agent.node_manager.nodes_dict.nearest_neighbors(robot_location.tolist(), 1)[0]
        node_coords = nearest_node.data.coords
        robot_node_location = node_coords

    # print(robot_location, robot_node_location)
    agent.update_planning_state(map_info, robot_node_location)
    agent.update_key_node_observation()
    
    agent.publish_node_markers()
    # print(agent.key_node_coords)
    env.step(action_in)
