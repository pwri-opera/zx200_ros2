# zx200_ros2
OPERA対応油圧ショベルzx200の土木研究所公開ROS2パッケージ群

## 概説
- 国立研究開発法人土木研究所が公開するOPERA（Open Platform for Eathwork with Robotics Autonomy）対応の油圧ショベルであるZX200用のROS2パッケージ群である
- 本パッケージに含まれる各launchファイルを起動することで、実機やシミュレータを動作させるのに必要なROSノード群が立ち上がる
- 動作確認済のROS Version : ROS2 Humble Hawksbil + Ubuntu 22.04 LTS

## ビルド方法
- ワークスペースの作成（既にwsを作成済の場合は不要．以下、新規作成するワークスペースの名称を"ros2_ws"と仮定して表記）
  ```bash
  $ cd ~/
  $ mkdir --parents ros2_ws/src
  $ cd ros2_ws
  $ colcon build 
  ```
- ~/ros2_ws/以下にbuild, install, log, srcディレクトリが作成される

- 依存パッケージ群をインストールした上でパッケージのビルドと自分のワークスペースをインストール環境上にOverlayする  
  [vcstoolに関する参考サイト](https://qiita.com/strv/items/dbde72e20a8efe62ef95)
  ```bash
  $ cd ~/ros2_ws/src
  $ git clone https://github.com/pwri-opera/zx200_ros2.git
  $ sudo apt update
  $ sudo apt install python3-rosdep2 
  $ rosdep update
  $ rosdep install -i --from-path src --rosdistro humble -y 
  <!--
  $ git clone https://github.com/strv/vcstool-utils.git
  $ ./vcstool-utils/import_all.sh -s .rosinstall ~/catkin_ws/src
  -->
  $ colcon build --symlink-install 
  $ . install/setup.bash
  ```

## 含有するサブパッケージ
### zx200_bringup:
- zx200の実機を動作させる際に必要なノード群を一括起動するためのlaunch用のサブパッケージ

### zx200_control:
- [ros_control](http://wiki.ros.org/ros_control)の枠組みに倣い、作業機（=swing_joint, boom_joint, arm_joint, bucket_joint, bucket_end_joint）の部分をupper_arm_"command_interface名"_contoller(JointTrajectoryController)という名称で実装したサブパッケージ
- command interfaceをvelocityとしての作業機での動作は未確認

### zx200_description:
- zx200用のロボットモデルファイル(dae, xacro含む)群

### zx200_unity:
- zx200をunityシミュレータ(OperaSim-AGX, OperaSim-PhysX)上で動作させるのに必要なノード群を一括起動するためのlaunch用のサブパッケージ

### zx200_moveit_config:
- zx200の作業機（=swing, boom, arm, bucketの4軸）のモーション制御のための設定ファイル群
- [MoveIt2](https://moveit.ros.org/)に準拠しMoveIt Setup Assistantを用いて作成
- command interfaceごとに，.urdf.xacro, ros2_controllers.yamlが存在

### zx200_unity:
- OPERAのUnityシミュレータ(OperaSim-AGX, OperaSim-PhysX)と連携するために必要なノード群を一括起動するためのlaunch用のサブパッケージ

## 各ROSノード群の起動方法
- 実機動作に必要なROS2ノード群の起動方法

  ```bash
  # 作業機のセットアップが完了した状態で
  $ ros2 launch zx200_bringup vehicle.launch.py
  ```
- Unityシミュレータとの連携に必要なROS2ノード群の起動方法
  ```bash
  $ ros2 launch zx200_unity zx200_standby.launch.py
  ```

## ハードウェアシステム
zx200のハードウェアのシステム構成を以下のブロック図へ示します
<!-- ![MicrosoftTeams-image (1)](https://github.com/pwri-opera/zx200_ros2/assets/24404939/a49534cc-13b1-461f-9368-152daabae51e) -->

## ソフトウェアシステム
### roslaunch zx200_unity zx200_standy.launch.py実行時のノード/トピックパイプライン（rqt_graph）
<!-- ![rosgraph_ros2_sim](https://github.com/pwri-opera/zx200_ros2/assets/24404939/1192aea7-bae1-4220-b8fc-18c0c0e2e3b1) -->

### roslaunch zx200_bringup zx200_vehicle.launch.py実行時のノード/トピックパイプライン（rqt_graph）  
注）zx200実機上でのみ実行可能です
<!-- ![rosgraph](https://github.com/pwri-opera/zx200_ros2/assets/24404939/7cb2ddb1-da25-43c3-8b22-58f838081da4) -->
