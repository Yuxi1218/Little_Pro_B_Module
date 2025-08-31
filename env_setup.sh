#!/bin/bash

# 先更新
sudo apt update && sudo apt upgrade -y
sudo apt install xfce4 xfce4-goodies firefox thunar xfce4-terminal -y
sudo apt-get install git neovim -y
sudo snap install btop

## ROS Noetic環境
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install -y ros-noetic-desktop-full python3-catkin-tools python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-pip

## ROS 相關套件(僅為X4考量)

### SLAM 部分

# Hector SLAM（推薦用於YDLidar X4）
sudo apt-get install ros-noetic-hector-slam -y

# Gmapping SLAM
sudo apt-get install ros-noetic-slam-gmapping -y

### 導航部分

# ROS Navigation Stack
sudo apt-get install ros-noetic-navigation -y

# 額外的路徑規劃器
sudo apt-get install ros-noetic-dwa-local-planner -y
sudo apt-get install ros-noetic-teb-local-planner -y

# AMCL定位
sudo apt-get install ros-noetic-amcl -y

### 視覺化工具

# 地圖儲存工具
sudo apt-get install ros-noetic-map-server -y

# 視覺化增強
sudo apt-get install ros-noetic-rqt-* -y
sudo apt-get install ros-noetic-plotjuggler-ros -y

### teleop-twist-keyboard
sudo apt-get install ros-noetic-teleop-twist-keyboard -y

## rosdep and workspace
sudo rosdep init
rosdep update

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin build

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# zsh
source /opt/ros/noetic/setup.zsh
source ~/catkin_ws/devel/setup.zsh

echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc
echo "source ~/catkin_ws/devel/setup.zsh" >> ~/.zshrc

## rosdep 補齊依賴
rosdep install --from-paths src --ignore-src -r -y

## ydlidar固定
# 設定USB裝置別名
sudo chmod 777 /dev/ttyUSB0
# 或建立udev規則讓裝置固定為 /dev/ydlidar
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"' | sudo tee /etc/udev/rules.d/ydlidar.rules

sudo udevadm control --reload-rules && sudo udevadm trigger