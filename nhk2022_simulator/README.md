# nhk2023_simulator
## これは何

nhk2023の環境をgazeboで再現するパッケージ。
今年度はメカナムなので特に物理シミュレーションをする意味が無いのでロジック確認のためのみ。

### 注記

物理シミュレーションを行わないので[こちら](http://cir-kit.github.io/blog/2015/02/19/gazebo-gazebo-plugins-in-ros/)に書かれたPlanar Move Pluginを用いて`/cmd_vel`を与えて直接モデルを動かしている。適切な慣性にしないとバランスを崩してしまうので低重心高重量にしてあります。

## ロボットモデル

現在は以下の2つに対応

- 簡易的な差動2輪モデル
- 簡易的な4輪オムニモデル (推奨)



## Launch

- フィールド+簡易的な4輪オムニモデル

  - R2のシミュレーターを起動
    ```
    roslaunch nhk2023_simulator simulation_R2.launch
    ```



- フィールド+簡易的な差動2輪モデル

  ```shell
  roslaunch nhk2023_simulator diff_drive_simulation.launch
  ```



上記launchを実行後

- R2

```shell
roslaunch nhk2023_launcher control_R2.launch
```

のロボット起動launchをたたくとロボットのシミュレーションが可能
