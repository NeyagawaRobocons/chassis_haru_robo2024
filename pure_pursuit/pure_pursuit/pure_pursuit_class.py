import numpy as np

class PurePursuit:
    def __init__(
            self, 
            speed: float, 
            lookahead_distance: float,
        ) -> None:
        self.speed = speed # 速度 [m/s]
        self.lookahead_distance = lookahead_distance # 前方参照距離 [m]

    def calculate_lookahead_point(
            self, 
            path_points: np.ndarray, # 経路ポイント
            current_point: np.ndarray, # 現在地
            tangents: np.ndarray # 接線ベクトル
        ) -> np.ndarray:
        """
        経路上の現在地における前方参照点を計算するメソッド
        """
        closest_point = None
        closest_tangent = None
        min_distance = float('inf')

        for path_point, tangent in zip(path_points, tangents):
            to_path_vector = path_point - current_point
            distance_to_path = np.linalg.norm(to_path_vector)

            # ロボットの現在位置からの距離がルックアヘッド距離より短い場合、スキップ
            if distance_to_path > self.lookahead_distance:
                continue

            # 現在位置からwaypointへのベクトルとwaypointの接ベクトルとの内積
            dot_product = np.dot(to_path_vector, tangent)

            # 内積が正で、かつ最短距離であれば更新
            if dot_product > 0 and distance_to_path < min_distance:
                min_distance = distance_to_path
                closest_point = path_point
                closest_tangent = tangent

        # まだ先行点が見つかっていない場合、最も近い点を使用
        if closest_point is None:
            return None

        # 接ベクトルとルックアヘッド距離を用いて先行点を補間
        t = (self.lookahead_distance - min_distance) / np.linalg.norm(closest_tangent)
        lookahead_point = closest_point + t * closest_tangent

        return lookahead_point
    
    def compute_velocity_vector(
            self, 
            path_points: np.ndarray, # 経路ポイント
            tangents: np.ndarray, # 接線ベクトル
            current_point: np.ndarray, # 現在地
            ) -> np.ndarray:
        # 経路上の先行点を見つける
        lookahead_point = self.calculate_lookahead_point(path_points, current_point, tangents)
        if lookahead_point is None:
            return None  # 先行点が見つからない場合

        # 速度ベクトルを計算する（先行点への単位ベクトルに速度を乗じる）
        direction_vector = lookahead_point - current_point
        unit_direction_vector = direction_vector / np.linalg.norm(direction_vector)
        velocity_vector = unit_direction_vector * self.speed
        return velocity_vector