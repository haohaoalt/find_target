<!--
 * @Author: zhanghao
 * @Date: 2022-07-09 09:22:43
 * @LastEditTime: 2022-07-09 11:00:47
 * @FilePath: /find_target/README.md
 * @Description: how to run
-->
# find_target
未知仿真环境中的物体识别与导航

## 1. 安装
```bash
cd find_target
sudo apt-get install ros-melodic-slam-gmapping
sudo apt-get install ros-melodic-navigation
cd mbot_voice/libs
sudo cp libmsc.so /usr/lib/
sudo apt install sox
sudo apt install libsox-fmt-all
```
```bash
cd find_target
catkin_make
```
## 2. 运行
```bash
cd find_target
source devel/setup.bash
roslaunch mbot_gazebo mbot_maze_gazebo.launch
roslaunch mbot_navigation exploring_slam_demo.launch
roslaunch mbot_vison find_target_pro.launch
```
```bash
git config --global user.email "haohaoalt@163.com"
git config --global user.name "hao"
```