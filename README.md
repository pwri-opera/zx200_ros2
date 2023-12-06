# zx200_ros2
OPERA 対応油圧ショベル zx200 の土木研究所公開 ROS2 パッケージ群

## 概説
- 国立研究開発法人土木研究所が公開する OPERA（Open Platform for Eathwork with Robotics Autonomy）対応の油圧ショベル zx200 用の ROS2 パッケージ群
- 本パッケージに含まれる各 launch ファイルを起動することで，実機やシミュレータを動作させるのに必要なROSノード群が起動
- 動作環境: ROS2 Humble Hawksbil + Ubuntu 22.04 LTS

## ビルド方法
- ワークスペースの作成（既にwsを作成済の場合は不要．
以下、新規作成するワークスペースの名称を `ros2_ws` と仮定して表記）
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
- excavator_com3_ros を除く，zx200 の実機を動作させる際に必要なノード群を一括起動するための launch 用サブパッケージ


### zx200_control:
- [ros_control](http://wiki.ros.org/ros_control) の枠組みに倣い，
作業機 (=swing_joint, boom_joint, arm_joint, bucket_joint, bucket_end_joint) 
の hardware interface を upper_arm_"command_interface名"_hardware という
名称で実装したサブパッケージ

> **TODO:** command interfaceをvelocityとしての作業機での動作は未確認

> **TODO:** upper_arm_unity_hardware の名前を使用している command interface がわかるように修正（現状は position）．

> **TODO:** OperaSim-PhysX が JointCmd 型に対応次第，upper_arm_unity_hardware を修正．

### zx200_description:
- zx200 用のロボットモデルファイル (dae, xacro含む) 群

### zx200_unity:
- zx200をunityシミュレータ(OperaSim-AGX, OperaSim-PhysX)上で動作させるのに必要なノード群を一括起動するためのlaunch用のサブパッケージ

### zx200_moveit_config:
- zx200の作業機（swing, boom, arm, bucketの4軸）のモーション制御のための設定ファイル群
- [MoveIt2](https://moveit.ros.org/)に準拠しMoveIt Setup Assistantを用いて作成したコンフィグファイルがベース
- command interfaceごとに，`zx200_~.urdf.xacro`, `ros2_~_controllers.yaml`が存在


## 各ROSノード群の起動方法
### 実機動作（Rviz上で目標姿勢を決定）の場合

1. 車載PCにRDP接続

2. 車載PCにてターミナルを起動し，以下のコマンドを実行

  ```bash
  # 作業機のセットアップ(油圧ロック解除まで)が完了した状態で
  ros2 launch zx200_bringup vehicle.launch.py
  ```
> **注:** rvizを使用する場合，RDP接続した車載PCにてターミナルを起動し実行する．SSH 接続だとlaunchの実行時にエラーが発生．

3. Rviz 上で目標姿勢を定め，`plan & execute` ボタンで軌道を計画・実行

### OperaSim の場合
- [OperaSim-AGX](https://github.com/pwri-opera/OperaSim-AGX), [OperaSim-PhysX](https://github.com/pwri-opera/OperaSim-PhysX) の README を参照
  <!-- ```bash
  ros2 launch zx200_unity zx200_standby.launch.py
  ``` -->
  <!-- **ターミナル1:**
  ```bash
  ros2 launch zx200_bringup opera_sim_unity.launch.py
  ``` -->

## ハードウェアシステム
> **TODO:** zx200のハードウェアのシステム構成を追加


## ソフトウェアシステム
### opera_sim_unity.launch.py 実行時のノード/トピックパイプライン（rqt_graph）
> **TODO:** opera_sim_unity.launch.py 実行時の rqt_graph のキャプチャを追加

### vehicle.launch.py実行時のノード/トピックパイプライン（rqt_graph）  
**注:** zx200実機上でのみ実行可能

> **TODO:** vehicle.launch.py 実行時の rqt_graph のキャプチャを追加
