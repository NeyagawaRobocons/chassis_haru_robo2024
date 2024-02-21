# outline of pure_puresuit_node.py
## input
- `/robot_pose` (topic):
  - description: get the pose of robot by tf
  - node: `localize_node`
  - type: geometry_msgs/msg/PoseStamped

- `/path_and_feedback` (action):
  - description: 経路データと経路上の特定の点のインデックスを渡し、特定の点を通過したというフィードバックをもらう。ゴールしたら結果が返ってくる。
  - node: `robot_master_node`s
  - type: robot_master/msg/PathAndFeedback

## output
- `/robot_vel` (topic):
  - description:
  - type: geometry_msgs/msg/Twist

## original message types
- `pure_pursuit/Path2DWithAngles`:
  - description: 経路データと経路上の特定の点のインデックスを渡す
  - field:
    - pure_pursuit/PointAndAngle[] path_with_angles

- `pure_pursuit/PointAndAngle`:
  - description: 経路データと経路上の特定の点のインデックスを渡す
  - field:
    - float32 x
    - float32 y
    - float32 angle

- `robot_master/PathAndFeedback`:
  - description: 経路データと経路上の特定の点のインデックスを渡し、特定の点を通過したというフィードバックをもらう。ゴールしたら結果が返ってくる。
  - field:

```
# パスとフィードバックのデータ

# ゴール定義
pure_pursuit/Path2DWithAngles path_with_angles
int32[] feedback_indices
---
# 結果定義
int32 final_index
---
# フィードバック定義
int32 current_index
```

## parameters
<!-- 表で示す -->

| name | type | default | description |
| ---- | ---- | ------- | ----------- |
| `speed` | float | 1.0 | ロボットの速さ [m/s] |
| `lookahead_distance` | float | 0.5 | 先行点までの距離 [m] |
| `path_p_gain` | float | 0.5 | 経路法線方向のP制御ゲイン |
| `angle_p_gain` | float | 0.1 | 角度Pゲイン |
| `angle_i_gain` | float | 0.01 | 角度Iゲイン |
| `distance_threshold` | float | 0.2 | 距離のしきい値 [m] |

## dependency
- `rclpy`
- `numpy`
- `geometry_msgs`
- `tf2_ros`
- `action_msgs`
- `pure_pursuit`
- `robot_master`


## algorithm

```python

#!/usr/bin/env python3
import numpy as np
from numpy.typing import NDArray
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from pure_pursuit.msg import Path2DWithAngles

class PurePursuitNode(Node):
    def __init__ () -> None:
        # トピック, アクションの初期化
        self.pose_sub = self.create_subscription(PoseStamped, '/robot_pose', self.pose_callback, 10)
        self.path_and_feedback_action
        self.vel_pub = self.create_publisher(Twist, '/robot_vel', 10)
        # パラメータの初期化
        ロボットの速さ, 1.0 # [m/s]
        先行点までの距離, 0.5 #[m]
        経路法線方向のP制御ゲイン: 0.5
        角度Pゲイン, 0.1
        角度Iゲイン, 0.01
        距離のしきい値, 0.2 # [m]
        # メンバ変数の初期化
        # パラメータ
        ロボットの速さ: float # [m/s]
        先行点までの距離: float # [m]
        経路法線方向のP制御ゲイン: float
        角度Pゲイン: float
        角度Iゲイン: float
        距離のしきい値: float # [m]
        # 動的変数
        経路データ: NDArray[np.float32] # np.array([[x, y, theta], [x, y, theta]])の形
        特定の点のインデックス: NDArray[np.int8] # np.array([1, 2, 3])の形
        接ベクトル: NDArray[np.float32] # np.array([[0.0, 0.0], [0.0, 0.0]])の形
        法線ベクトル: NDArray[np.float32] # np.array([[0.0, 0.0], [0.0, 0.0]])の形
        自己位置: NDArray[np.float32] # np.array([x, y, theta])の形

    def アクションコールバック () -> None:
        経路データの受け取りと格納
        特定の点のインデックスの受け取りと格納
        接ベクトルの計算と格納(経路データ)
        法線ベクトルの計算と格納(接ベクトル)

    def pose_callback () -> None:
        自己位置の格納
        速度入力の決定(自己位置, 経路データ, 接ベクトル, 法線ベクトル)
        Twistに格納
        速度入力パブリッシュ
        if 最も近い経路上の点のインデックス == 特定の点のインデックス:
            フィードバックにインデックスを返す
        elif 距離(自己位置, 経路の終点) < 距離のしきい値:
            完了処理を行う

    def 速度入力の決定 (
            自己位置: NDArray[np.float32], 
            経路データ: NDArray[np.float32], 
            接ベクトル: NDArray[np.float32],
            法線ベクトル: NDArray[np.float32]
        ) -> None:
        先行点の計算 (自己位置, 経路データ, 先行点までの距離)
        最も近い経路上の点の計算 (自己位置, 経路データ)
        pure pursuit速度入力の計算 (自己位置, 先行点, 先行点までの距離, ロボットの速さ)
        経路法線方向のP制御入力の計算 (自己位置, 最も近い経路上の点)
        角度PI制御入力の計算 (自己位置, 最も近い経路上の点)

    def 先行点の計算 (
            自己位置: NDArray[np.float32], 
            経路データ: NDArray[np.float32], 
            先行点までの距離: float
        ) -> NDArray[np.float32]:

    def 最も近い経路上の点の計算 (自己位置: NDArray[np.float32], 経路データ: NDArray[np.float32]) -> int:

    def pure_pursuit速度入力の計算 (
            自己位置: NDArray[np.float32], 
            先行点: NDArray[np.float32], 
            先行点までの距離: float, 
            ロボットの速さ: float
        ) -> NDArray[np.float32]:

    def 経路法線方向のP制御入力の計算 (自己位置: NDArray[np.float32], 最も近い経路上の点: int) -> NDArray[np.float32]:

    def 角度PI制御入力の計算 (自己位置: NDArray[np.float32], 最も近い経路上の点: int) -> NDArray[np.float32]:

    def 接ベクトルの計算 (経路データ: NDArray[np.float32]) -> NDArray[np.float32]:
        # 接ベクトルの計算

    def 法線ベクトルの計算 (接ベクトル: NDArray[np.float32]) -> NDArray[np.float32]:
        # 法線ベクトルの計算

def main () -> None:
    rclpy.init()
    node = PurePursuitNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

# path_generator_node.py
### ノード名: path_generator_node

### 概要
`path_generator_node`は、指定された特定のポイントとそれらのポイントでのロボットのメカニズムコマンドからパスデータを生成し、ベジエ曲線でこれらを補間するノードです。補間されたパスは、より滑らかなロボットの動きを実現するために使用されます。

### パラメーター
1. **特定のポイント**
   - **形式**: ROSパラメータ (YAMLファイル) からのリスト
   - **内容**: 特定のポーズ（x, y, theta）と、そのポイントでのコマンドを含むリスト。
   - **例**: 
   ```
    [
      [x, y, theta, command: MechaState], 
      [x, y, theta, command: MechaState], 
      [x, y, theta, command: MechaState]
    ]
   ```

2. **出力YAMLファイルパス**
   - **説明**: 生成されたパスデータを保存するYAMLファイルのパス。

### 出力
- **ファイル**: 指定されたYAMLファイルに以下の情報を保存します。
  - **パス**: 補間されたポイントのリスト。各ポイントは `[x, y, theta]` の形式。
  - **インデックス**: 特定のポーズとコマンドのポイントに対応するインデックスのリスト。
  - **コマンド**: コマンドのリスト。各コマンドは、特定のポイントでのロボットのメカニズムの状態を指示します。

### コマンドの種類 (`mecha_control/msg/MechaState.msg`)
- `byte daiza_state`: 展開(1)、回収(2)、設置(3)、格納(4)
- `byte hina_state`: 同上
- `bool bonbori_state`: ぼんぼりオフ(false)、ぼんぼり点灯(true)

### 実装の要点
- 特定のポイントとコマンドを入力として受け取り、ベジエ曲線を用いてパスを補間する。
- 補間されたパスと関連情報をYAMLファイルに保存する。
- ROS 2のパラメータ機能を活用して入力を読み込み、指定されたパスに結果を出力する。