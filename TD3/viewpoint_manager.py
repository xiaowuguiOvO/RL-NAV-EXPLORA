import rospy
import numpy as np
from geometry_msgs.msg import PointStamped
from node_manager import NodeManager
from utils import get_updating_node_coords, get_frontier_in_map
from nav_msgs.msg import OccupancyGrid
from utils import MapInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class ViewpointManager:
    def __init__(self):
        # 初始化节点管理器
        self.node_manager = NodeManager()
        self.current_viewpoint = None  # 当前最佳视点
        self.viewpoints = []  # 候选视点列表
        self.map_info = None

        # ROS 发布器
        self.cur_viewpoint_pub = rospy.Publisher('/current_viewpoint', PointStamped, queue_size=1)
        self.viewpoints_pub = rospy.Publisher('/candidate_viewpoints', MarkerArray, queue_size=10)

        # 订阅 occupancy map
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

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

    def generate_viewpoints(self, robot_location):
        """
        使用 Occupancy Map 和机器人位置生成候选视点。
        """
        if self.map_info is None:
            rospy.logwarn("Map info is not available yet. Skipping viewpoint generation.")
            return

        # 生成候选节点
        candidate_nodes, free_connected_map = get_updating_node_coords(robot_location, self.map_info, check_connectivity=False)
        # 检测前沿点
        frontiers = get_frontier_in_map(self.map_info)

        # 计算每个候选节点的效用值
        self.viewpoints = []
        for node in candidate_nodes:
            utility = self.calculate_utility(node, frontiers)
            # if utility > 0:  # 只保留有探索价值的节点
            self.viewpoints.append((node, utility))

        # 按效用值排序
        self.viewpoints.sort(key=lambda x: x[1], reverse=True)


    def calculate_utility(self, node, frontiers):
        """
        计算节点的效用值，基于覆盖的前沿点数量。
        """
        utility = 0
        for frontier in frontiers:
            distance = np.linalg.norm(np.array(node) - np.array(frontier))
            if distance <= self.map_info.cell_size * 2:  # 假设效用范围为 2 个栅格
                utility += 1
        return utility

    # def select_best_viewpoint(self):
    #     """
    #     从候选视点中选择最佳视点。
    #     """
    #     if self.viewpoints:
    #         self.current_viewpoint = self.viewpoints[0][0]  # 选择效用值最高的视点
    #     else:
    #         self.current_viewpoint = None

    # def publish_candidate_viewpoints(self):

    #     for viewpoint, utility in self.viewpoints:
    #         viewpoint_msg = PointStamped()
    #         viewpoint_msg.header.stamp = rospy.Time.now()
    #         viewpoint_msg.header.frame_id = "map"
    #         viewpoint_msg.point.x = viewpoint[0]
    #         viewpoint_msg.point.y = viewpoint[1]
    #         viewpoint_msg.point.z = 0.0
    #         self.viewpoint_pub.publish(viewpoint_msg)


    def publish_viewpoints_as_markers(self, viewpoints_with_utility):
        """
        将带有效用值的候选视点作为 MarkerArray 发布。
        颜色会根据效用值从红（低）到绿（高）进行渐变。
        :param viewpoints_with_utility: 一个列表，每个元素是 ((x, y), utility) 形式的元组。
        """
        marker_array = MarkerArray()

        # 如果列表为空，发布一个“全部删除”的指令来清除 RViz 中的旧标记
        if not viewpoints_with_utility:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.action = Marker.DELETEALL
            marker_array.markers.append(marker)
            self.viewpoints_pub.publish(marker_array)
            return

        # --- 计算效用值的范围，用于颜色归一化 ---
        utilities = [vp[1] for vp in viewpoints_with_utility]
        min_util = min(utilities)
        max_util = max(utilities)
        util_range = max_util - min_util

        # --- 遍历每个带有效用值的视点 ---
        for i, (point, utility) in enumerate(viewpoints_with_utility):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "candidate_viewpoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 0.9

            # --- 根据效用值计算颜色 ---
            normalized_utility = 0.5  # 默认值（如果所有点效用值相同）
            if util_range > 1e-6:  # 避免除以零
                normalized_utility = (utility - min_util) / util_range
            
            # normalized_utility 从 0 到 1，颜色从红到绿
            marker.color.r = 1.0 - normalized_utility
            marker.color.g = normalized_utility
            marker.color.b = 0.0
            
            marker.lifetime = rospy.Duration()
            marker_array.markers.append(marker)

        self.viewpoints_pub.publish(marker_array)

    def update(self, robot_location):
        """
        更新候选视点并发布最佳视点。
        """
        self.generate_viewpoints(robot_location)
        # self.select_best_viewpoint()
        self.publish_viewpoints_as_markers(self.viewpoints)
