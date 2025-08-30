# 安裝指南

## 先更新

~~我原本想寫不能先放調理包~~

```terminal
sudo apt update && sudo apt upgrade -y
```

## 桌面

本次選擇Xfce4雖然我私心想用i3-wm  

```terminal
sudo apt install xfce4 xfce4-goodies firefox thunar xfce4-terminal -y

```

## 一些輔助跟必要的東西(看個人)

```terminal
sudo apt-get install git zsh neovim -y
sudo snap install btop
```

```terminal
# zsh 插件
sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k

git clone https://github.com/marlonrichert/zsh-autocomplete ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autocomplete

git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions

git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting

```

```terminal
nano ~/.zshrc
```

找到並新增

```terminal
ZSH_THEME="powerlevel10k/powerlevel10k"
plugins=(git zsh-autocomplete zsh-autosuggestions zsh-syntax-highlighting)
```

## ROS Noetic環境

```terminal
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl -y # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install -y     ros-noetic-desktop-full     python3-catkin-tools     python3-rosdep     python3-rosinstall     python3-rosinstall-generator     python3-wstool     build-essential     python3-pip
```

## ROS 相關套件(僅為X4考量)

### SLAM 部分

```terminal
# Hector SLAM（推薦用於YDLidar X4）
sudo apt-get install ros-noetic-hector-slam -y

# Gmapping SLAM
sudo apt-get install ros-noetic-slam-gmapping -y
```

### 導航部分

```terminal
# ROS Navigation Stack
sudo apt-get install ros-noetic-navigation -y

# 額外的路徑規劃器
sudo apt-get install ros-noetic-dwa-local-planner -y
sudo apt-get install ros-noetic-teb-local-planner -y

# AMCL定位
sudo apt-get install ros-noetic-amcl -y
```

### 視覺化工具

```terminal
# 地圖儲存工具
sudo apt-get install ros-noetic-map-server -y

# 視覺化增強
sudo apt-get install ros-noetic-rqt-* -y
sudo apt-get install ros-noetic-plotjuggler-ros -y
```

### teleop-twist-keyboard

```terminal
sudo apt-get isntall ros-noetic-teleop-twist-keyboard -y
```

## rosdep and workspace

```terminal
sudo rosdep init
rosdep update

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin build


source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

```terminal
# zsh
source /opt/ros/noetic/setup.zsh
source ~/catkin_ws/devel/setup.zsh

echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc
echo "source ~/catkin_ws/devel/setup.zsh" >> ~/.zshrc
```

## rosdep 補齊依賴

```terminal
rosdep install --from-paths src --ignore-src -r -y
```
