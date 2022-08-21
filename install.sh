#!/bin/bash

echo "#############################################"
echo "#### Installing nhk2023_ilias repository ####"
echo "#############################################"

## Resolve dependencies (from git repository)
echo "1/4 Resolve dependencies (from git repository)"
sudo apt-get update
sudo apt-get install python3-vcstool #依存関係の解決に便利だぞ
dir_path=`pwd`
dirs=`find $dir_path -maxdepth 2 -type f -name *.rosinstall`
echo $dirs
for dir in $dirs;
do
    echo $dir
    vcs import ../ < $dir
done
echo "エラーを吐く場合はgithubからhttp経由でのリクエストの為ワンタイムパスワードを生成する必要があります"

## Resolve dependencies (from apt repository)
echo "2/4 Resolve dependencies (from apt repository)"
rosdep update
rosdep install -i --from-paths -y ../roswww
dirs=`find $dir_path -maxdepth 1 -type d`

for dir in $dirs;
do
    echo $dir
    rosdep install -i --from-paths -y $dir
done

# sudo apt-get install -y ros-noetic-realsense2-description
# sudo apt-get install -y ros-noetic-robot-localization
# sudo apt-get install -y ros-noetic-navigation
# sudo apt-get install -y ros-noetic-rplidar-ros

## Give permission to python scripts
echo "3/4 Give permissions to python scripts"
dirs=`find $dir_path -maxdepth 3 -type f -name *.py`

for dir in $dirs;
do
    chmod +x $dir
    echo $dir
done

## Give permission to action msg
chmod +x ./bezier_path_planning_pursuit/action/PursuitPath.action

## install pyrealsense2
sudo apt install python3-pip
# pip install pyrealsense2

## Build
echo "4/4 catkin build"
catkin build

echo "Installing finished successfully."