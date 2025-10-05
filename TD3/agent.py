import time
from copy import deepcopy

import numpy as np
import torch
import matplotlib.pyplot as plt
import copy
import matplotlib.colors as colors
import parameter
import rospy
from utils import *
# from parameter import *
from node_manager import NodeManager
from visualization_msgs.msg import MarkerArray, Marker
from parameter import *
class Agent:
    def __init__(self, device='cpu', plot=False):
        self.device = device
        self.policy_net = None
        self.plot = plot

        # location and map
        self.location = None
        self.map_info = None

        # map related parameters
        self.cell_size = parameter.CELL_SIZE
        self.node_resolution = parameter.NODE_RESOLUTION
        self.updating_map_size = parameter.UPDATING_MAP_SIZE

        # map and updating map
        self.map_info = None
        self.updating_map_info = None

        # frontiers
        self.frontier = set()

        # node managers
        self.node_manager = NodeManager()

        # graph
        self.node_coords, self.utility, self.guidepost = None, None, None
        self.current_index, self.adjacent_matrix, self.neighbor_indices = None, None, None

        # rarefied graph
        self.key_node_coords, self.key_utility, self.key_guidepost = None, None, None
        self.key_current_index, self.key_adjacent_matrix, self.key_neighbor_indices = None, None, None

        self.node_publisher = rospy.Publisher('/all_nodes', MarkerArray, queue_size=10)
        
    def update_map(self, map_info):
        self.map_info = map_info

    def update_updating_map(self, location):
        # the updating map is the part of the global map that maybe affected by new measurements
        self.updating_map_info = self.get_updating_map(location)

    def update_location(self, location):
        self.location = location
        node = self.node_manager.nodes_dict.find(location.tolist())
        if self.node_manager.nodes_dict.__len__() == 0:
            pass
        else:
            node.data.set_visited()

    def update_frontiers(self):
        self.frontier = get_frontier_in_map(self.updating_map_info)

    def get_updating_map(self, location):
        # the map includes all nodes that may be updating
        updating_map_origin_x = (location[
                                     0] - self.updating_map_size / 2)
        updating_map_origin_y = (location[
                                     1] - self.updating_map_size / 2)

        updating_map_top_x = updating_map_origin_x + self.updating_map_size
        updating_map_top_y = updating_map_origin_y + self.updating_map_size

        min_x = self.map_info.map_origin_x
        min_y = self.map_info.map_origin_y
        max_x = (self.map_info.map_origin_x + self.cell_size * (self.map_info.map.shape[1] - 1))
        max_y = (self.map_info.map_origin_y + self.cell_size * (self.map_info.map.shape[0] - 1))

        if updating_map_origin_x < min_x:
            updating_map_origin_x = min_x
        if updating_map_origin_y < min_y:
            updating_map_origin_y = min_y
        if updating_map_top_x > max_x:
            updating_map_top_x = max_x
        if updating_map_top_y > max_y:
            updating_map_top_y = max_y

        updating_map_origin_x = (updating_map_origin_x // self.cell_size + 1) * self.cell_size
        updating_map_origin_y = (updating_map_origin_y // self.cell_size + 1) * self.cell_size
        updating_map_top_x = (updating_map_top_x // self.cell_size) * self.cell_size
        updating_map_top_y = (updating_map_top_y // self.cell_size) * self.cell_size

        updating_map_origin_x = np.round(updating_map_origin_x, 1)
        updating_map_origin_y = np.round(updating_map_origin_y, 1)
        updating_map_top_x = np.round(updating_map_top_x, 1)
        updating_map_top_y = np.round(updating_map_top_y, 1)

        updating_map_origin = np.array([updating_map_origin_x, updating_map_origin_y])
        updating_map_origin_in_global_map = get_cell_position_from_coords(updating_map_origin, self.map_info)

        updating_map_top = np.array([updating_map_top_x, updating_map_top_y])
        updating_map_top_in_global_map = get_cell_position_from_coords(updating_map_top, self.map_info)

        updating_map = self.map_info.map[
                       updating_map_origin_in_global_map[1]:updating_map_top_in_global_map[1] + 1,
                       updating_map_origin_in_global_map[0]:updating_map_top_in_global_map[0] + 1]

        updating_map_info = MapInfo(updating_map, updating_map_origin_x, updating_map_origin_y, self.cell_size)
        # print(f"--- DEBUG: 切割出的局部地图，尺寸: {updating_map.shape} ---")

        return updating_map_info

    def update_planning_state(self, map_info, location):
        t1 = time.time()
        self.update_map(map_info)
        self.update_location(location)

        self.update_updating_map(self.location)
        self.update_frontiers()
        self.location = self.node_manager.update_graph(self.location,
                                       self.frontier,
                                       self.updating_map_info,
                                       self.map_info) # 花好多时间啊 想想怎么优化一下
        t2 = time.time()
        print("update frontiers and graph", t2 - t1)
        
        t1 = time.time()
        self.node_manager.get_rarefied_graph(self.location, self.map_info)
        t2 = time.time()
        # print("graph rarefaction", t2 - t1)
        # self.node_coords, self.utility, self.guidepost, self.adjacent_matrix, self.current_index, self.neighbor_indices = \
        #     self.update_padding_observation()
        self.key_node_coords, self.key_utility, self.key_guidepost, self.key_adjacent_matrix, self.key_current_index, self.key_neighbor_indices = \
            self.update_key_node_observation()
        t1 = time.time()
        # self.node_coords, self.utility, self.guidepost, self.adjacent_matrix, self.current_index, self.neighbor_indices = \
        #     self.update_observation()
        t2 = time.time()
        # print("update observation", t2 - t1)
        # print("update key node graph", t2 - t1)

    def update_observation(self):
        all_node_coords = []
        for node in self.node_manager.nodes_dict.__iter__():
            all_node_coords.append(node.data.coords)
        all_node_coords = np.array(all_node_coords).reshape(-1, 2)
        utility = []
        guidepost = []

        n_nodes = all_node_coords.shape[0]
        adjacent_matrix = np.ones((n_nodes, n_nodes)).astype(int)
        node_coords_to_check = all_node_coords[:, 0] + all_node_coords[:, 1] * 1j
        for i, coords in enumerate(all_node_coords):
            node = self.node_manager.nodes_dict.find((coords[0], coords[1])).data
            utility.append(node.utility)
            guidepost.append(node.visited)
            for neighbor in node.neighbor_set:
                index = np.argwhere(node_coords_to_check == neighbor[0] + neighbor[1] * 1j)
                assert index is not None
                index = index[0][0]
                adjacent_matrix[i, index] = 0

        utility = np.array(utility)
        guidepost = np.array(guidepost)

        current_index = np.argwhere(node_coords_to_check == self.location[0] + self.location[1] * 1j)[0][0]
        neighbor_indices = np.argwhere(adjacent_matrix[current_index] == 0).reshape(-1)
        return all_node_coords, utility, guidepost, adjacent_matrix, current_index, neighbor_indices

    def update_key_node_observation(self):
        all_key_node_coords = []
        for key_node_coords in self.node_manager.key_node_dict.keys():
            all_key_node_coords.append(np.array(key_node_coords))
        all_key_node_coords = np.array(all_key_node_coords).reshape(-1, 2)
        utility = []
        guidepost = []

        n_nodes = all_key_node_coords.shape[0]
        adjacent_matrix = np.ones((n_nodes, n_nodes)).astype(int)
        node_coords_to_check = all_key_node_coords[:, 0] + all_key_node_coords[:, 1] * 1j
        for i, coords in enumerate(all_key_node_coords):
            node = self.node_manager.key_node_dict[(coords[0], coords[1])]
            utility.append(node.utility)
            guidepost.append(node.visited)
            for neighbor in node.neighbor_set:
                neighbor = np.array([neighbor[0], neighbor[1]])
                index = np.argwhere(node_coords_to_check == neighbor[0] + neighbor[1] * 1j)
                index = index[0][0]
                adjacent_matrix[i, index] = 0

        utility = np.array(utility)
        guidepost = np.array(guidepost)

        current_index = np.argwhere(node_coords_to_check == self.location[0] + self.location[1] * 1j)[0][0]
        neighbor_indices = np.argwhere(adjacent_matrix[current_index] == 0).reshape(-1)

        return all_key_node_coords, utility, guidepost, adjacent_matrix, current_index, neighbor_indices

    
    def get_pandding_observation(self):
        node_coords = self.key_node_coords
        node_utility = self.key_utility.reshape(-1, 1)
        node_guidepost = self.key_guidepost.reshape(-1, 1)
        current_index = self.key_current_index
        edge_mask = self.key_adjacent_matrix
        current_edge = self.key_neighbor_indices
        n_node = node_coords.shape[0]

        current_node_coords = node_coords[self.key_current_index]
        node_coords = np.concatenate((node_coords[:, 0].reshape(-1, 1) - current_node_coords[0],
                                            node_coords[:, 1].reshape(-1, 1) - current_node_coords[1]),
                                           axis=-1) / UPDATING_MAP_SIZE
        node_utility = node_utility / (SENSOR_RANGE * 3.14 // FRONTIER_CELL_SIZE)
        node_inputs = np.concatenate((node_coords, node_utility, node_guidepost), axis=1)
        node_inputs = torch.FloatTensor(node_inputs).unsqueeze(0).to(self.device)

        assert node_coords.shape[0] < NODE_PADDING_SIZE, print(node_coords.shape[0], NODE_PADDING_SIZE)
        padding = torch.nn.ZeroPad2d((0, 0, 0, NODE_PADDING_SIZE - n_node))
        node_inputs = padding(node_inputs)

        node_padding_mask = torch.zeros((1, 1, n_node), dtype=torch.int16).to(self.device)
        node_padding = torch.ones((1, 1, NODE_PADDING_SIZE - n_node), dtype=torch.int16).to(
            self.device)
        node_padding_mask = torch.cat((node_padding_mask, node_padding), dim=-1)

        current_index = torch.tensor([current_index]).reshape(1, 1, 1).to(self.device)

        edge_mask = torch.tensor(edge_mask).unsqueeze(0).to(self.device)

        padding = torch.nn.ConstantPad2d(
            (0, NODE_PADDING_SIZE - n_node, 0, NODE_PADDING_SIZE - n_node), 1)
        edge_mask = padding(edge_mask)

        current_in_edge = np.argwhere(current_edge == self.key_current_index)[0][0]
        current_edge = torch.tensor(current_edge).unsqueeze(0)
        k_size = current_edge.size()[-1]
        padding = torch.nn.ConstantPad1d((0, K_SIZE - k_size), 0)
        current_edge = padding(current_edge)
        current_edge = current_edge.unsqueeze(-1)

        edge_padding_mask = torch.zeros((1, 1, k_size), dtype=torch.int16).to(self.device)
        edge_padding_mask[0, 0, current_in_edge] = 1
        padding = torch.nn.ConstantPad1d((0, K_SIZE - k_size), 1)
        edge_padding_mask = padding(edge_padding_mask)
        
        observation = {
            'node_inputs': node_inputs,
            'node_padding_mask': node_padding_mask,
            'edge_mask': edge_mask,
            'current_index': current_index,
            'next_edge': current_edge,
            'edge_padding_mask': edge_padding_mask
        }
        return observation

    
    def get_observation(self, robot_location):

        node_coords = deepcopy(self.key_node_coords)
        node_utility = self.key_utility.reshape(-1, 1)
        node_guidepost = self.key_guidepost.reshape(-1, 1)
        current_index = self.key_current_index
        edge_mask = self.key_adjacent_matrix
        current_edge = self.key_neighbor_indices

        node_coords[current_index] = robot_location

        current_node_coords = robot_location
        node_coords = np.concatenate((node_coords[:, 0].reshape(-1, 1) - current_node_coords[0],
                                      node_coords[:, 1].reshape(-1, 1) - current_node_coords[1]),
                                     axis=-1) / parameter.UPDATING_MAP_SIZE / 2
        node_utility = node_utility / (parameter.UTILITY_RANGE * 3.14 // parameter.FRONTIER_CELL_SIZE)
        node_inputs = np.concatenate((node_coords, node_utility, node_guidepost), axis=1)
        node_inputs = torch.FloatTensor(node_inputs).unsqueeze(0).to(self.device)

        edge_mask = torch.tensor(edge_mask).unsqueeze(0).to(self.device)

        current_in_edge = np.argwhere(current_edge == current_index)[0][0]
        current_edge = torch.tensor(current_edge).unsqueeze(0).to(self.device)
        k_size = current_edge.size()[-1]
        current_edge = current_edge.unsqueeze(-1)

        current_index = torch.tensor([current_index]).reshape(1, 1, 1).to(self.device)

        edge_padding_mask = torch.zeros((1, 1, k_size), dtype=torch.int16).to(self.device)
        edge_padding_mask[0, 0, current_in_edge] = 1
        observation = {
            'node_inputs': node_inputs,
            'edge_mask': edge_mask,
            'current_index': current_index,
            'next_edge': current_edge,
            'edge_padding_mask': edge_padding_mask
        }
        print(node_inputs.shape, edge_mask.shape, current_index.shape, current_edge.shape, edge_padding_mask.shape)
        return observation
        # return [node_inputs, None, edge_mask, current_index, current_edge, edge_padding_mask]

    def get_next_observation(self, next_node_index, observation):
        node_inputs, _, edge_mask, curren_index, _, _ = observation
        next_edge = torch.nonzero(edge_mask[0, next_node_index] == 0).flatten()
        next_in_edge = torch.nonzero(next_edge == next_node_index).item()
        curren_in_edge = torch.nonzero(next_edge == curren_index.item()).item()
        k_size = next_edge.size()[-1]
        next_edge = next_edge.unsqueeze(-1).unsqueeze(0)
        next_node_index = torch.tensor([next_node_index]).reshape(1, 1, 1).to(self.device)
        edge_padding_mask = torch.zeros((1, 1, k_size), dtype=torch.int16).to(self.device)
        edge_padding_mask[0, 0, next_in_edge] = 1
        edge_padding_mask[0, 0, curren_in_edge] = 1
        
        observation = {
            'node_inputs': node_inputs,
            'edge_mask': edge_mask,
            'current_index': next_node_index,
            'next_edge': next_edge,
            'edge_padding_mask': edge_padding_mask
        }
        print(node_inputs.shape, edge_mask.shape, next_node_index.shape, next_edge.shape, edge_padding_mask.shape)
        return observation
        # return [node_inputs, None, edge_mask, next_node_index, next_edge, edge_padding_mask]

    def select_next_waypoint(self, observation, greedy=True):
        _, _, _, _, current_edge, _ = observation
        
        # Start inference timing
        inference_start = time.time()
        with torch.no_grad():
            logp = self.policy_net(*observation)
        inference_time = time.time() - inference_start
        
        # Start post-processing timing
        postprocess_start = time.time()
        if greedy:
            action_index = torch.argmax(logp, dim=1).long()
        else:
            action_index = torch.multinomial(logp.exp(), 1).long().squeeze(1)
        next_node_index = current_edge[0, action_index.item(), 0].item()
        next_position = self.key_node_coords[next_node_index]
        postprocess_time = time.time() - postprocess_start
        
        # Log timing information
        g = "\033[92m"  # green
        n = "\033[0m"   # reset color
        rospy.loginfo(f"Inference Time - Network: {g}{inference_time*1000:.2f}ms{n}, "
                     f"Post-process: {g}{postprocess_time*1000:.2f}ms{n}")

        return next_position, next_node_index

    def plot_env(self, step, robot_location):
        # quite slow, only use it to debug

        # plt.switch_backend('TKAgg')
        plt.ion()
        plt.clf()

        plt.subplot(1, 2, 1)
        nodes = get_cell_position_from_coords(self.key_node_coords, self.map_info).reshape(-1, 2)
        if len(self.frontier) > 0:
            frontiers = get_cell_position_from_coords(np.array(list(self.frontier)), self.map_info).reshape(-1, 2)
            plt.scatter(frontiers[:, 0], frontiers[:, 1], c='r', s=2)
        robot = get_cell_position_from_coords(robot_location, self.map_info)
        # plt.imshow(self.map_info.map, cmap='gray')
        plt.imshow(self.map_info.map + 1.1, cmap='gray_r', norm=colors.LogNorm())
        plt.axis('off')
        plt.scatter(nodes[:, 0], nodes[:, 1], c=self.key_utility, zorder=2)
        for node, utility in zip(nodes, self.key_utility):
            plt.text(node[0], node[1], str(utility), zorder=3)
        plt.plot(robot[0], robot[1], 'mo', markersize=16, zorder=5)
        for coords in self.key_node_coords:
            node = self.node_manager.key_node_dict[(coords[0], coords[1])]
            for neighbor_coords in node.neighbor_set:
                end = (np.array(neighbor_coords) - coords) / 2 + coords
                plt.plot((np.array([coords[0], end[0]]) - self.map_info.map_origin_x) / self.cell_size,
                         (np.array([coords[1], end[1]]) - self.map_info.map_origin_y) / self.cell_size, 'tan', zorder=1)

        plt.subplot(1, 2, 2)
        nodes = get_cell_position_from_coords(self.node_coords, self.map_info)
        if len(self.frontier) > 0:
            frontiers = get_cell_position_from_coords(np.array(list(self.frontier)), self.map_info).reshape(-1, 2)
            plt.scatter(frontiers[:, 0], frontiers[:, 1], c='r', s=2)
        robot = get_cell_position_from_coords(robot_location, self.map_info)
        plt.imshow(self.map_info.map + 1.1, cmap='gray_r', norm=colors.LogNorm())
        plt.axis('off')
        plt.scatter(nodes[:, 0], nodes[:, 1], c=self.utility, zorder=2)
        for node, utility in zip(nodes, self.utility):
            plt.text(node[0], node[1], str(utility), zorder=3)
        plt.plot(robot[0], robot[1], 'mo', markersize=16, zorder=5)
        for coords in self.node_coords:
            node = self.node_manager.nodes_dict.find(coords.tolist()).data
            for neighbor_coords in node.neighbor_set:
                end = (np.array(neighbor_coords) - coords) / 2 + coords
                plt.plot((np.array([coords[0], end[0]]) - self.map_info.map_origin_x) / self.cell_size,
                         (np.array([coords[1], end[1]]) - self.map_info.map_origin_y) / self.cell_size, 'tan', zorder=1)

        plt.pause(1e-3)

        plt.savefig('{}/{}_samples.png'.format(f'gifs', step), dpi=150)
        # plt.close()

    def publish_node_markers(self):
        """
        在 RViz 中发布所有节点的 MarkerArray。
        """
        marker_array = MarkerArray()

        # --- 开始修改 ---
        # 1. 找到 utility 的最大值和最小值
        # if self.utility.size == 0:  # 检查数组的元素数量是否为 0
        if self.key_utility.size == 0:
            return

        min_utility = min(self.key_utility)
        max_utility = max(self.key_utility)
        utility_range = max_utility - min_utility

        # 防止所有值都相同时除以零
        if utility_range == 0:
            utility_range = 1.0
        # --- 结束修改 ---

        for i, coords in enumerate(self.key_node_coords):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time(0)
            marker.ns = "nodes"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = coords[0]
            marker.pose.position.y = coords[1]
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            # --- 开始修改 ---
            # 2. 计算当前 utility 的归一化值 (0.0 到 1.0 之间)
            normalized_utility = (self.key_utility[i] - min_utility) / utility_range
            
            # 3. 使用归一化后的值来设置颜色
            marker.color.a = 1.0
            marker.color.r = normalized_utility  # 红色分量现在会动态变化
            marker.color.g = 0.5
            marker.color.b = 1.0 - normalized_utility # 蓝色分量也会相应变化
            # --- 结束修改 ---
            
            marker_array.markers.append(marker)

        self.node_publisher.publish(marker_array)
        
    def reset(self):
        """
        重置 Agent 的所有内部状态，为新的 episode 做准备。
        """
        rospy.loginfo("Resetting Agent state (clearing map and nodes)...")
        
        # 重新初始化所有在 episode 之间需要清空的状态变量
        self.location = None
        self.map_info = None
        self.updating_map_info = None

        self.frontier = set()

        # 这是最关键的一步：创建一个全新的 NodeManager，丢弃旧的节点和图
        self.node_manager = NodeManager()

        # 将所有图相关的变量也重置为 None
        self.node_coords, self.utility, self.guidepost = None, None, None
        self.current_index, self.adjacent_matrix, self.neighbor_indices = None, None, None
        
        self.key_node_coords, self.key_utility, self.key_guidepost = None, None, None
        self.key_current_index, self.key_adjacent_matrix, self.key_neighbor_indices = None, None, None
