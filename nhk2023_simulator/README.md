# nhk2023_simulator
## これは何

nhk2023の環境をgazeboで再現するパッケージ。

### 注記

物理シミュレーションを行わないので[こちら](https://github.com/gazebosim/gz-sim)に書かれた`ignition::gazebo::systems::VelocityControl`を用いて`/cmd_vel`を与えて直接モデルを動かしています。現状ではこのプラグインを使うと恐らく重力が働いていない？ので要改善。

## ロボットモデル

- ERの簡易シミュレーション



## Launch

- フィールド+簡易的なERモデル

  - R2のシミュレーターを起動
    ```
    ros2 launch nhk2023_simulator simulation_ER.py
    ```


    上記launchを実行後

  - ER

    ```shell
    ros2 launch nhk2023_launcher control_ER.launch
    ```

    のロボット起動launchをたたくとロボットのシミュレーションが可能
