# outline of pure_puresuit_node.py
## input
## output
## parameters
## algorithm

```python

#!/usr/bin/env python3
import numpy as np
from numpy.typing import NDArray
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from pure_pursuit.msg import Path2DWithAngles

class 経路追従クラス:
    def 初期化:
        トピック，TF，アクションの初期化
        パラメータの初期化
            ロボットの速さ, 1.0 # [m/s]
            先行点までの距離, 0.5 #[m]
            経路法線方向のP制御ゲイン: 0.5
            角度Pゲイン, 0.1
            角度Iゲイン, 0.01
            距離のしきい値, 0.2 # [m]
        メンバ変数の初期化
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

    def アクションコールバック:
        経路データの受け取りと格納
        特定の点のインデックスの受け取りと格納
        接ベクトルの計算と格納(経路データ)
        法線ベクトルの計算と格納(経路データ)

    def 自己位置TFコールバック:
        自己位置の格納
        速度入力の決定(自己位置, 経路データ, 接ベクトル, 法線ベクトル)
        Twistに格納
        速度入力パブリッシュ
        if 最も近い経路上の点のインデックス == 特定の点のインデックス:
            フィードバックにインデックスを返す
        elif 距離(自己位置, 経路の終点) < 距離のしきい値:
            完了処理を行う

    def 速度入力の決定(自己位置, 経路データ, 接ベクトル, 法線ベクトル):
        先行点の計算(自己位置, 経路データ, 先行点までの距離)
        最も近い経路上の点の計算(自己位置, 経路データ)
        pure pursuit速度入力の決定(自己位置, 先行点, 先行点までの距離, ロボットの速さ)
        経路法線方向のP制御入力の決定(自己位置, 最も近い経路上の点)
        角度PI制御入力の決定

    def 先行点の計算:

    def 最も近い経路上の点の計算:

    def pure pursuit速度入力の決定:

    def 経路法線方向のP制御入力の決定:

    def 角度PI制御入力の決定:

    def 接ベクトルの計算:

    def 法線ベクトルの計算:

```