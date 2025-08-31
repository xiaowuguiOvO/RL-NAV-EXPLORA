#!/bin/zsh

# 设置 ROS 环境变量
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
export ROS_PORT_SIM=11311

# 设置 Gazebo 路径
export GAZEBO_RESOURCE_PATH=~/Desktop/nav/DRL-robot-navigation/catkin_ws/src/multi_robot_scenario/launch

# 加载 zsh 配置（如果有需要）
source ~/.zshrc

# 进入工作空间并加载 ROS 环境
cd ../catkin_ws
source devel_isolated/setup.zsh

cd ../TD3
conda activate ctsac