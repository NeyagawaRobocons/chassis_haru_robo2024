#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mecha_control.msg import MechaState
import numpy as np
from numpy.typing import NDArray
import json
import yaml
import matplotlib.pyplot as plt

class PathGeneratorNode(Node):
    def __init__(self):
        super().__init__('path_generator')
        self.declare_parameter('path_density', 20)
        self.declare_parameter('output_file', 'path.yaml')

        self.path_density = self.get_parameter('path_density').value
        # その他のパラメータ宣言
        self.declare_parameter('poses', "[[0.0, 0.0, 0.0]]")  # デフォルト値を適切な形式で指定

        # posesパラメータを文字列として取得し、JSONとして解析
        poses_str = self.get_parameter('poses').value
        self.get_logger().info(f"poses_str: {poses_str}")
        self.poses = json.loads(poses_str)

        self.output_file = self.get_parameter('output_file').value

        self.get_logger().info(f"path_density: {self.path_density}")
        self.get_logger().info(f"poses: {self.poses}")
        self.get_logger().info(f"output_file: {self.output_file}")
        self.get_logger().info("path_generator_node has been started")

    def generate_and_save_path(self) -> None:
        # description: generate path and save it to a file
        specified_points = np.array([[item[0], item[1]] for item in self.poses])
        specified_angles = np.array([item[2] for item in self.poses])
        path = self.generate_path(points=specified_points)
        indices = self.extract_indices(path, specified_points)
        angles = self.generate_angles(path, indices, specified_angles)
        commands: MechaState = self.commands_to_MechaState(self.commands)
        self.save_path_to_yaml(path, angles, indices, commands, self.output_file)
        self.get_logger().info("path_generator_node has saved path to a file")
        self.plot_path(path, angles, indices, commands)
    
    def save_path_to_yaml(self, 
            path: NDArray[np.float64],
            angles: NDArray[np.float64],
            indices: NDArray[np.int64],
            commands: NDArray[MechaState],
            file_name
        ) -> None:
        # description: save path to a yaml file
        data = {
            "path": path,
            "angles": angles,
            "indices": indices,
            "commands": [command.to_dict() for command in commands]  # MechaStateメッセージを辞書に変換
        }
        with open(file_name, 'w') as file:
            yaml.dump(data, file)

    def generate_path(self, 
            points: NDArray[np.float64], 
            radius: float = 0.3, 
            resolution: int = 20
        ) -> NDArray[np.float64]:
        # description: generate 2D path from points in poses_and_commands
        path: NDArray[np.float64] = np.zeros((0, 2))

        # 最初のベジェ曲線のスタート点を設定
        p0 = points[0]  # 最初の点
        p1 = points[1]  # 二番目の点

        # p0からp1へのベクトル
        vec0 = p1 - p0
        norm_vec0 = vec0 / np.linalg.norm(vec0)

        # ベジェ曲線の最初の制御点を計算（ここでは、p1からの距離に基づく）
        first_bezier_start = p0 + norm_vec0 * radius

        # 最初の直線セグメントをpath_x, path_yに追加
        line_x_values = np.linspace(p0[0], first_bezier_start[0], num=20, endpoint=True)
        line_y_values = np.linspace(p0[1], first_bezier_start[1], num=20, endpoint=True)
        path = np.append(path, np.column_stack((line_x_values, line_y_values)), axis=0)

        for i in range(1, len(points)-1):
            p0 = points[i - 1]
            p1 = points[i]
            p2 = points[i + 1]

            # 前後の点からのベクトル
            vec1 = (p1 - p0) / np.linalg.norm(p1 - p0)
            vec2 = (p2 - p1) / np.linalg.norm(p2 - p1)

            # 制御点1 (終点の前の制御点)
            ctrl1 = p1 - vec1 * radius

            # ベジェ曲線の制御点
            ctrl2 = p1 - vec1 * radius * 0.5
            ctrl3 = p1 + vec2 * radius * 0.5
            ctrl4 = p1 + vec2 * radius

            # ベジェ曲線部分の生成
            bezier_x, bezier_y = self.generate_bezier_curve(ctrl1, ctrl2, ctrl3, ctrl4, resolution)
            path = np.append(path, np.column_stack((bezier_x, bezier_y)), axis=0)

        # 最後の直線部分を追加
        final_line_start = ctrl4
        final_line_end = points[-1]
        line_x_values = np.linspace(final_line_start[0], final_line_end[0], num=20, endpoint=True)
        line_y_values = np.linspace(final_line_start[1], final_line_end[1], num=20, endpoint=True)
        path = np.append(path, np.column_stack((line_x_values, line_y_values)), axis=0)

        return path

    def generate_bezier_curve(self, 
            p0, p1, p2, p3, # 4つの制御点 (np.array([x, y])型)
            resolution: int = 20
        ) -> NDArray[np.float64]:
        t_values = np.linspace(0, 1, num=resolution)
        points: NDArray[np.float64] = np.zeros((resolution, 2))

        for i, t in enumerate(t_values):
            x = (1-t)**3 * p0[0] + 3*(1-t)**2 * t * p1[0] + 3*(1-t) * t**2 * p2[0] + t**3 * p3[0]
            y = (1-t)**3 * p0[1] + 3*(1-t)**2 * t * p1[1] + 3*(1-t) * t**2 * p2[1] + t**3 * p3[1]
            points[i] = np.array([x, y])

        return points
    
    def generate_angles(self,
            path: NDArray[np.float64],
            indices: NDArray[np.int32], 
            specified_angles: NDArray[np.float64]
        ) -> NDArray[np.float64]:
        # pathの各ポイントに対する角度データを生成
        angles = np.zeros(len(path))
        for i in range(len(indices) - 1):
            # indices[i]からindices[i+1]までの間の角度を補間
            start_index = indices[i]
            end_index = indices[i + 1]
            # 指定された角度から次の角度への補間を行い、その区間の長さに合わせて解像度を調整
            angles[start_index:end_index] = np.linspace(
                specified_angles[i], specified_angles[i + 1], num=end_index - start_index)
        
        # 最後の指定された角度を、経路の残りの部分に適用
        if indices[-1] < len(path):
            angles[indices[-1]:] = specified_angles[-1]

        return angles

    def interpolate_angle(self,
            start_angle: float = 0.0,
            end_angle: float = 0.0,
            resolution: int = 20
        ) -> NDArray[np.float64]:
        # description: interpolate angle through all points in poses_and_commands
        return np.linspace(start_angle, end_angle, num=resolution)

    def extract_indices(self, 
            path: NDArray[np.float64], 
            specified_points: NDArray[np.float64]
        ) -> NDArray[np.int32]:
        indices = np.array([
            np.argmin(np.linalg.norm(path - point, axis=1))
            for point in specified_points
        ])
        return indices

    def commands_to_MechaState(self, poses_and_commands) -> MechaState:
        # poses_and_commandsリストからコマンド部分を抽出
        commands = [item[3] for item in poses_and_commands]  # item[3]はコマンドを指します
        commands = [MechaState(**command) for command in commands]  # 辞書をMechaStateメッセージに変換
        return commands

    def plot_path(self, path, angles, indices, commands, file_name='path.png'):
        """
        description: 
        - plot path with angles, indices and commands
        - save the plot as a png file
        how to plot:
        - plot path
        - plot angles as arrows
        - plot indices as points and display the index number
        - plot commands as text on the points of path whose index is in indices
        """

def main(args=None):
    rclpy.init(args=args)
    path_generator_node = PathGeneratorNode()
    rclpy.spin(path_generator_node)
    # シャットダウン処理
    path_generator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()