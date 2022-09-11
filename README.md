<img src="https://keiorogiken.files.wordpress.com/2018/12/e382abe383a9e383bc.png?w=2160" width="50%"/>


**_Lets aim for CAMBODIA_**


[![ROS: Noetic](https://img.shields.io/badge/ROS-Humble-brightgreen)](http://wiki.ros.org/humble)  [![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0) [![KRA: ilias](https://img.shields.io/badge/KRA-ilias-blue.svg)](https://keiorogiken.wordpress.com/)

# nhk2023_ilias

**NHK Robocon 2023** project  team "**ilias**" (KeioRoboticsAssociation:robot:)

## Install

Before run `install.sh`, please install Nodejs
Clone this repository and just run `install.sh`

```shell
cd ~/rogi_ws/src
git clone git@github.com:KeioRoboticsAssociation/nhk2023_ilias.git
cd nhk2023_ilias
source install.sh
```

[Here](https://github.com/KeioRoboticsAssociation/nhk2023_ilias/blob/main/Dependencies.md) is a detail description about installing and dependencies of this repository.

### issues

We are using https link while doing `git clone` from .rosinstall files and this is not recommended.




## 各パッケージについて

- [bezier_path_planning_pursuit](https://github.com/KeioRoboticsAssociation/nhk2023_ilias/blob/main/bezier_path_planning_pursuit/README.md)

  経路計画、追従を担当するパッケージ

- [joy_commander](https://github.com/KeioRoboticsAssociation/nhk2023_ilias/blob/main/joy_commander/README.md)

  Joyコンで速度指令値を送れるパッケージ

- [nhk2022_launcher](https://github.com/KeioRoboticsAssociation/nhk2023_ilias/blob/main/nhk2023_launcher/README.md)

  ロボットを起動するlaunchファイルが入っているパッケージ

- [nhk2022_simulator](https://github.com/KeioRoboticsAssociation/nhk2023_ilias/blob/main/nhk2023_simulator/README.md)

  Gazeboによるシミュレーターを構築するためのファイルが入っているパッケージ

- [nhk2022_webgui](https://github.com/KeioRoboticsAssociation/nhk2023_ilias/blob/main/nhk2023_webgui/README.md)

  WebGUIを起動するためのファイルが入っているパッケージ



## シミュレーターの簡単な遊び方

1. フィールドとロボットモデルのリスポーン


  ```
  roslaunch nhk2023_simulator simulation_R2.launch
  ```

2. コントローラーの起動

  ```shell
  roslaunch nhk2023_launcher control_R2.launch
  ```


  キーコンフィグ

  <img src="key_config.png" width="100%"/>


3. WebGUIのリンクは[こちら](http://localhost:8085/nhk2023_webgui/WebGUI.html)


## Lisence

The applications are licensed under GPLv3 license.
