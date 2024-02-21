# chassis_haru_robo2024
これは春ロボ2024の足回り自動制御用ROSパッケージである。
## 目標
- 最速でフィールド内を動ける自動制御の足回りを作る！
## 小目標
- 第零段階：とりあえず動くようにする
  - 自己位置推定：オドメトリ
  - モータの速度制御：PID制御@マイコン
- 第一段階：ある程度決まった位置へ動けるようにする -> ver.1システム
  - 経路：点to点(PTP)
  - 経路追従：PID制御(入力：各タイヤの速度, 出力：x, y, theta)
- 第二段階：壁を避けてフィールド内を動けるようにする -> ver.2システム
  - 自己位置推定：LiDARを追加
  - 経路：連続した直線の経路
  - 経路追従：Pure Pursuit法＋ポテンシャル？
- 第三段階：最速で動けるようにする
  - 経路：最適経路

# システムについて
## システム構成
![image](https://github.com/NeyagawaRobocons/chassis_haru_robo2024/assets/143268535/c24b4813-8417-4854-8a19-b02d182cb452)

## 実装するノードについて
- odometry_node：
  - 役割：オドメトリを計算するノード
  - 入力：/odometer_3wheelトピック (nucleo_agent/OdometerData型：**自作型**)：計測輪の角変位
  - 出力1：/odometry_poseトピック (geometry_msgs/PoseStamped型)：計算した自己位置
  - 出力2：/odomフレームと/base_linkフレームのtf：amclなどに渡すときの自己位置
- localize_node (名称は適宜変更)：
  - 役割：2D LiDARなどでオドメトリを補正するノード
  - 入力：/odometry_poseトピック  (geometry_msgs/PoseStamped型)
  - 出力：/robot_poseトピック (geometry_msgs/Pose型)：補正された自己位置
    -  -> 型は変更される可能性アリ
- path_server (※**第二段階から実装**)：
  - 役割：目標の経路(姿勢情報、速度情報込み)をトピックに投げるノード
  - 入力：未定、計算するかも
  - 出力：/robot_pathトピック (chassis_haru_robo2024/OrientedPath型：**自作型**)：経路
- calc_vel (名称は適宜変更)：
  - 役割：経路情報と自己位置を比較して、速度指令を計算するノード
  - 第零目標時点(※**図には入ってない**)：
    - 処理：目標点と現在位置に対してPI制御を行う (操作量：各タイヤの角速度、制御量：x, y, \theta)
    - 入力：/target_poseトピック (geometry_msgs/PoseStamped型)：目標点の座標
    - 出力：/input_velトピック (std_msgs/Float64Array型)：各タイヤの目標角速度 (長さ3の配列)
  - @第二目標時点：
    - 処理：Pure Pursuit法で目標速度を計算
    - 入力：/robot_pathトピック (chassis_haru_robo2024/OrientedPath型：**自作型**)：経路
    - 出力：/input_velトピック (std_msgs/Float64Array型)：各タイヤの目標角速度 (長さ3の配列)
   
## 自作のトピック型について
- nucleo_agent/OdometerData型
  - 説明：オドメトリを計算するための計測輪の角変位を提供する。nucleoと通信するパッケージ (nucleo_agent)内で実装する。
```
# 計測輪の角変位のデータ [rad]

std_msgs/Header header
float64[3] rotation # 各計測輪の角変位
float64[3] angular_vel # 各計測輪の角速度
```

- chassis_haru_robo2024/PoseWithVel型
  - 説明：chassis_haru_robo2024/OrientedPath型を定義するための型。ある瞬間の位置・姿勢・速度ベクトルとタイムスタンプを提供する
```
# 位置・姿勢・速度ベクトル

geometry_msgs/Pose pose
geometry_msgs/Vector3 velocity
```

- chassis_haru_robo2024/OrientedPath型
  - 説明：位置・姿勢・速度ベクトルを含んだ経路を配列で提供する
```
# 位置・姿勢・速度ベクトルを含んだ経路

std_msgs/Header header
PoseWithVel[ ] poses
```

## dependencies
- laser_filters
  - url: https://github.com/ros-perception/laser_filters.git
  - branch: ros2
- emcl2
  - url: https://github.com/CIT-Autonomous-Robot-Lab/emcl2_ros2.git
  - branch: main
- ldlidar
  - url: https://github.com/Pylgos/ldlidar.git
  - branch: ros2
- nav2
- nucleo_agent
  - url: https://github.com/NeyagawaRobocons/nucleo_agent.git
  - branch: main
- mecha_control
  - url: https://github.com/NeyagawaRobocons/mecha_control.git
  - branch: main