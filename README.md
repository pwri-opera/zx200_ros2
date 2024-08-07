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

> **TODO:** upper_arm_unity_hardware の名前を使用している command interface がわかるように修正（現状は position）．

> **TODO:** OperaSim-PhysX が JointCmd 型に対応次第，upper_arm_unity_hardware を修正．

### zx200_description:
- zx200 用のロボットモデルファイル (dae, xacro含む) 群

### zx200_unity:
- zx200 を Unity シミュレータ (OperaSim-AGX, OperaSim-PhysX) 上で動作させるのに必要なノード群を一括起動するための launch 用のサブパッケージ

### zx200_moveit_config:
- zx200 の作業機（swing, boom, arm, bucketの4軸）のモーション制御のための設定ファイル群
- [MoveIt2](https://moveit.ros.org/)に準拠しMoveIt Setup Assistant を用いて作成したコンフィグファイルがベース
- command interface ごとに，`zx200_~.urdf.xacro`, `ros2_~_controllers.yaml`が存在

## 各ROSノード群の起動方法
### 実機動作（Rviz上で目標姿勢を決定）

1. 車載PCにRDP接続

2. 車載PCと自動運転コントローラのCANベースcom3通信開始（アライブカウンタ入力＋エンジンローアイドル指令）．以下のコマンドを実行
  ```bash
  ros2 launch zx200_com3_ros zx200_com3_ros.launch.py
  ```
3. ロック解除リモコンを用いてエンジン始動

4. com3_ros直下のsample_ros_cmd.shをfront_control_modeを指定して実行
   - front_control_mode=0:作業機のコントロールをdisable
   - front_control_mode=1:作業機をeffortコントロールモードでenable
   - front_control_mode=2:作業機をvelocityコントロールモードでenable

5. ロック解除リモコンを用いて油圧ロックを解除

6. 車載PCにてターミナルを起動し，以下のコマンドを実行

  ```bash
  # 作業機のセットアップ(油圧ロック解除まで)が完了した状態で実行
  # command_interface_nameは"effort"か"velocity"
  # use_rvizはTrueかFalse
  ros2 launch zx200_bringup vehicle.launch.py command_interface_name:=<commnad_interface_name> use_rviz:=<use_rviz>
  ```
> **注:** rvizを使用する場合，RDP接続した車載PCにてターミナルを起動する。SSH 接続だとlaunchの実行時にエラーが発生．

7. Rviz 上で目標姿勢を定め，`plan & execute` ボタンで軌道を計画・実行

### OperaSim と連携
- [OperaSim-AGX](https://github.com/pwri-opera/OperaSim-AGX), [OperaSim-PhysX](https://github.com/pwri-opera/OperaSim-PhysX) の README を参照
  <!-- ```bash
  ros2 launch zx200_unity zx200_standby.launch.py
  ``` -->

### Rviz 上でのみ表示
`mock_components/GenericSystem` を使用し，Rviz 上で zx200 を表示．move_group_interface等の動作確認時に使用．
  ```bash
  ros2 launch zx200_moveit_config fake_demo.launch.py
  ```

## ハードウェアシステム
zx200のハードウェアのシステム構成を以下のブロック図へ示します
<!-- ![MicrosoftTeams-image (1)](https://github.com/pwri-opera/zx200_ros2/assets/24404939/a49534cc-13b1-461f-9368-152daabae51e) -->
![zx200_hardware_system](https://github.com/pwri-opera/zx200_ros2/assets/24404939/f89d62a0-375a-4f22-9182-08c6cccc9756)

## ソフトウェアシステム
### ros2 launch zx200_moveit_config fake_demo.launch.py実行時のノード/トピックパイプライン（rqt_graph）
![rosgraph_zx200_demo_fake](https://github.com/pwri-opera/zx200_ros2/assets/24404939/9e75e0aa-3aa6-4ec2-ba27-f63a2867c351)

### ros2 launch zx200_unity zx200_standy.launch.py実行時のノード/トピックパイプライン（rqt_graph）
<!-- ![rosgraph_ros2_sim](https://github.com/pwri-opera/zx200_ros2/assets/24404939/1192aea7-bae1-4220-b8fc-18c0c0e2e3b1) -->
![rosgraph_zx200_unity_standby](https://github.com/pwri-opera/zx200_ros2/assets/24404939/e6823a08-bf39-4257-8ffa-6537a19d2407)

### ros2 launch zx200_bringup zx200_vehicle.launch.py実行時のノード/トピックパイプライン（rqt_graph）  
注）zx200実機上でのみ実行可能です
![zx200_ros2_effort_rosgraph](https://github.com/pwri-opera/zx200_ros2/assets/46485303/30f95979-99f7-4810-9ae2-e4ad261bb30b)
