from velodyne_env import GazeboEnv
from geometry_msgs.msg import Twist
import time
import rospy
from nav_msgs.msg import Odometry
from viewpoint_manager import ViewpointManager
import numpy as np
environment_dim = 20
env = GazeboEnv("test_simulator.launch", environment_dim)
viewpoint_manager = ViewpointManager()

try:
    rospy.wait_for_message('/r1/odom', Odometry, timeout=10)
    print("Odometry message received. Starting control loop.")
except rospy.ROSException:
    print("Did not receive odom message after 10 seconds. Exiting.")
    exit()

# vel_cmd = Twist()
# vel_cmd.linear.x = 0.5  # Set linear velocity
# vel_cmd.angular.z = 0.0  # Set angular velocity
while True:
    action_in = [0.5, 1]  # Example action input
    robot_location = env.get_robot_location()
    robot_location = np.array(robot_location)
    viewpoint_manager.update(robot_location)

    env.step(action_in)
