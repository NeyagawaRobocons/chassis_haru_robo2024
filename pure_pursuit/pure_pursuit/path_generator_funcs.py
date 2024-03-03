import numpy as np
from numpy.typing import NDArray
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.ndimage import gaussian_filter1d

def generate_bezier_curve(
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

def bezier_from_3_points(
        p0: NDArray[np.float64],
        p1: NDArray[np.float64],
        p2: NDArray[np.float64],
        resolution: int = 20,
        radius: float = 0.3
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
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
    return generate_bezier_curve(ctrl1, ctrl2, ctrl3, ctrl4, resolution), ctrl1, ctrl4

def generate_path(
        points: NDArray[np.float64],
        radiuses: NDArray[np.float64], # radius of each curve
        resolution: int = 20,
    ) -> NDArray[np.float64]:
    # description: generate 2D path from points in poses_and_commands
    path: NDArray[np.float64] = np.zeros((0, 2))

    # 最初のベジェ曲線のスタート点を設定
    p0 = points[0]  # 最初の点
    p1 = points[1]  # 二番目の点

    # p0からp1へのベクトル
    vec0 = p1 - p0
    norm_vec0 = vec0 / np.linalg.norm(vec0) # ベクトルの正規化

    # ベジェ曲線の最初の制御点を計算（ここでは、p1からの距離に基づく）
    first_bezier_start = p1 - norm_vec0 * radiuses[0]

    # 最初の直線セグメントをpathに追加
    line_x_values = np.linspace(p0[0], first_bezier_start[0], num=resolution, endpoint=True)
    line_y_values = np.linspace(p0[1], first_bezier_start[1], num=resolution, endpoint=True)
    path = np.append(path, np.column_stack((line_x_values, line_y_values)), axis=0)

    for i in range(1, len(points)-1):
        p0 = points[i - 1]
        p1 = points[i]
        p2 = points[i + 1]
        bezier, ctrl1, ctrl4 = bezier_from_3_points(p0, p1, p2, resolution, radiuses[i])

        if i != 1:
            line_x_values = np.linspace(pre_ctrl4[0], ctrl1[0], num=resolution, endpoint=True)
            line_y_values = np.linspace(pre_ctrl4[1], ctrl1[1], num=resolution, endpoint=True)
            path = np.append(path, np.column_stack((line_x_values, line_y_values)), axis=0)

        path = np.append(path, bezier, axis=0)

        pre_ctrl4 = ctrl4

    # 最後の直線部分を追加
    final_line_start = path[-1]
    final_line_end = points[-1]
    line_x_values = np.linspace(final_line_start[0], final_line_end[0], num=resolution, endpoint=True)
    line_y_values = np.linspace(final_line_start[1], final_line_end[1], num=resolution, endpoint=True)
    path = np.append(path, np.column_stack((line_x_values, line_y_values)), axis=0)

    return path

def generate_angles(
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

def interpolate_angle(
        start_angle: float = 0.0,
        end_angle: float = 0.0,
        resolution: int = 20
    ) -> NDArray[np.float64]:
    # description: interpolate angle through all points in poses_and_commands
    return np.linspace(start_angle, end_angle, num=resolution)

def extract_indices(
        path: NDArray[np.float64],
        specified_points: NDArray[np.float64]
    ) -> NDArray[np.int32]:
    indices = np.array([
        np.argmin(np.linalg.norm(path - point, axis=1))
        for point in specified_points
    ])
    return indices

def interpolate_speed(
        poses: NDArray[np.float64],
        path: NDArray[np.float64],
        radiuses: NDArray[np.float64],
        indices: NDArray[np.int32],
        max_speed: float,
        speed_rates: NDArray[np.float64]
    ) -> NDArray[np.float64]:
    speeds = np.zeros(len(path))
    curve_end_index = 0

    # plt.figure(figsize=(8, 6))

    for i in range(len(indices) - 2):
        before_curve_index = np.argmin(np.abs(np.linalg.norm(poses[i+1][:2] - path[:indices[i+1]], axis=1) - radiuses[i+1] * 2.5))
        if i != 0:
            speeds[curve_end_index:before_curve_index] = np.array([max_speed for _ in range(before_curve_index - curve_end_index)])
        else:
            speeds[curve_end_index:before_curve_index] = np.linspace(max_speed * speed_rates[i], max_speed * speed_rates[i+1], num=before_curve_index - curve_end_index)
        curve_index = np.argmin(np.abs(np.linalg.norm(poses[i+1][:2] - path[:indices[i+1]], axis=1) - radiuses[i+1]))
        speeds[before_curve_index:curve_index] = np.linspace(max_speed, max_speed * speed_rates[i+1], num=curve_index - before_curve_index)
        speeds[curve_index:indices[i+1]] = np.array([max_speed * speed_rates[i+1] for _ in range(indices[i+1] - curve_index)])
        curve_end_index = np.argmin(np.abs(np.linalg.norm(poses[i+1][:2] - path[indices[i+1]:indices[i+2]], axis=1) - radiuses[i+1])) + indices[i+1]
        if i != len(indices) - 3:
            speeds[indices[i+1]:curve_end_index] = np.linspace(max_speed * speed_rates[i+1], max_speed, num=curve_end_index - indices[i+1])
        else:
            speeds[indices[i+1]:curve_end_index] = np.linspace(max_speed * speed_rates[i+1], max_speed * speed_rates[i+1], num=curve_end_index - indices[i+1])

        # plt.axvline(x=before_curve_index, color='r', linestyle='--')
        # plt.axvline(x=curve_index, color='r', linestyle='--')
        # plt.axvline(x=curve_end_index, color='r', linestyle='--')

    # speeds[curve_end_index:] = max_speed * np.cos(np.pi * np.linspace(0, 1, num=len(speeds[curve_end_index:])))
    # speeds[curve_end_index:] = np.linspace(max_speed * speed_rates[-2], 0, num=len(speeds[curve_end_index:]))
    if speed_rates[-1] < 1.0:
        speeds[curve_end_index:] = max_speed * speed_rates[-2] * 2 * np.arccos(np.linspace(0, 1, num=len(speeds[curve_end_index:]))) / np.pi
    else:
        speeds[curve_end_index:] = np.linspace(max_speed * speed_rates[-2], max_speed * speed_rates[-1], num=len(speeds[curve_end_index:]))

    # plt.plot(speeds, label='Speeds')
    # plt.legend()
    # plt.show()

    return speeds

def interpolate_lookahead_distance(
        max_lookahead_distance: float,
        speeds: NDArray[np.float64],
        max_speed: float,
    ) -> NDArray[np.float64]:
    return max_lookahead_distance * np.sqrt(speeds / max_speed)

def interpolate_gains(
        max_p_gain: float,
        max_i_gain: float,
        poses: NDArray[np.float64],
        path: NDArray[np.float64],
        radiuses: NDArray[np.float64],
        indices: NDArray[np.int32],
        angle_p_rates: NDArray[np.float64],
        angle_i_rates: NDArray[np.float64],
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    p_gains = interpolate_speed(poses, path, radiuses, indices, max_p_gain, angle_p_rates)
    i_gains = interpolate_speed(poses, path, radiuses, indices, max_i_gain, angle_i_rates)
    return p_gains, i_gains

def interpolate_path_gains(
        path_p_gain: float,
        path_p_magnification: float,
        path_i_gain: float,
        angles: NDArray[np.float64],
        max_angle: float,
    ):
    # path_p_gains = path_p_gain * (1.0 + (1.0 / path_p_magnification - 1.0) * angles[:] / max_angle)
    path_p_gains = path_p_gain * np.ones(len(angles))
    path_i_gains = path_i_gain * np.ones(len(angles))

    return path_p_gains, path_i_gains

def draw_square(
        ax: plt.Axes, 
        center: float, length: float, yaw_rad: float, 
        color: str = 'b') -> None:    
    # 正方形の角の座標を計算
    half = length / 2
    corners = np.array([
        [-half, -half],
        [-half, half],
        [half, half],
        [half, -half],
        [-half, -half]  # 最初の点に戻るため
    ])
    
    # 回転行列を使用して角の座標を回転
    rotation_matrix = np.array([
        [np.cos(yaw_rad), -np.sin(yaw_rad)],
        [np.sin(yaw_rad), np.cos(yaw_rad)]
    ])
    rotated_corners = np.dot(corners, rotation_matrix)
    
    # 中心座標を加算して、正しい位置に移動
    rotated_corners[:, 0] += center[0]
    rotated_corners[:, 1] += center[1]
    
    # 正方形を描画
    ax.plot(rotated_corners[:, 0], rotated_corners[:, 1], color+'-')

def distance(p1: NDArray[np.float64], p2: NDArray[np.float64]) -> float:
    return np.linalg.norm(p1 - p2)

def generate_and_save_path(
        poses: NDArray[np.float64], 
        commands: NDArray[np.float64], 
        max_lookahead_distance: float, 
        max_speed: float, 
        distance_threshold: float,
        angle_threshold: float,
        path_p_gain: float,
        path_p_magnification: float,
        path_i_gain: float,
        resolution: int, 
        path_number: int,
        field_side: str = 'left'
    ) -> None:
    # description: generate path and save it to a file
    specified_points = np.array([[item[0], item[1]] for item in poses])
    specified_angles = np.array([item[2] for item in poses])
    radiuses = np.array([item[3] for item in poses])
    speed_rates = np.array([item[4] for item in poses])
    angle_p_gains = np.array([item[5] for item in poses])
    angle_i_gains = np.array([item[6] for item in poses])
    max_p_gain = np.max(angle_p_gains)
    max_i_gain = np.max(angle_i_gains)
    angle_p_rates = angle_p_gains / max_p_gain
    angle_i_rates = angle_i_gains / max_i_gain
    path = generate_path(points=specified_points, radiuses=radiuses, resolution=resolution)
    indices = extract_indices(path, specified_points)
    print(indices)
    angles = generate_angles(path, indices, specified_angles)
    # print(angles)
    max_angle = np.max(angles)
    speeds = interpolate_speed(poses, path, radiuses, indices, max_speed, speed_rates)
    # speeds = np.array([max_speed for _ in range(len(path))])
    speeds = gaussian_filter1d(speeds, sigma=3)
    lookahead_distances = interpolate_lookahead_distance(max_lookahead_distance, speeds, max_speed)
    p_gains, i_gains = interpolate_gains(max_p_gain, max_i_gain, poses, path, radiuses, indices, angle_p_rates, angle_i_rates)
    path_p_gains, path_i_gains = interpolate_path_gains(path_p_gain, path_p_magnification, path_i_gain, angles, max_angle)
    distance_threshold_mask = np.zeros(len(path))
    distance_threshold_mask[0] = distance_threshold
    angle_threshold_mask = np.zeros(len(path))
    angle_threshold_mask[0] = angle_threshold
    # save
    data_to_save = np.hstack((
        path, angles.reshape(-1, 1), 
        speeds.reshape(-1, 1), 
        lookahead_distances.reshape(-1, 1), 
        p_gains.reshape(-1, 1), 
        i_gains.reshape(-1, 1), 
        path_p_gains.reshape(-1, 1),
        path_i_gains.reshape(-1, 1),
        distance_threshold_mask.reshape(-1, 1),
        angle_threshold_mask.reshape(-1, 1),
        ))
    if len(commands) == len(poses):
        indices_and_commands = np.hstack((indices.reshape(-1, 1), commands))
    else:
        print("size of commands is not equal to size of poses")
        return

    path_file = f"../csv/path{path_number}.csv"
    indices_and_commands_file = f"../csv/indices_and_commands{path_number}.csv"

    print(f"Saving path to {path_file}")
    print(f"Saving indices and commands to {indices_and_commands_file}")

    # CSVファイルに保存
    np.savetxt(path_file, data_to_save, delimiter=",", fmt='%s', comments='', header='x,y,theta,speed,lookahead_distance,p_gain,i_gain,path_p_gain,path_i_gain,distance_threshold,angle_threshold')
    np.savetxt(indices_and_commands_file, indices_and_commands, delimiter=",", fmt='%s', comments='', header='index,command1,command2,command3,set_pose_flag')

    plt.figure(figsize=(10, 20))
    plt.quiver(path[:, 0], path[:, 1], -np.sin(angles), np.cos(angles), scale=20, color='r', alpha=0.5)
    plt.plot(path[:, 0], path[:, 1], 'b-', label='Path')
    # plt.scatter(path[:, 0], path[:, 1], color='g', zorder=5, label='Specified Points')
    plt.scatter(path[indices, 0], path[indices, 1], color='g', zorder=5, label='Specified Points')

    for i in range(len(indices) - 2):
        before_curve_index = np.argmin(np.abs(np.linalg.norm(poses[i+1][:2] - path[:indices[i+1]], axis=1) - radiuses[i+1] * 2.5))
        curve_index = np.argmin(np.abs(np.linalg.norm(poses[i+1][:2] - path[:indices[i+1]], axis=1) - radiuses[i+1]))
        curve_end_index = np.argmin(np.abs(np.linalg.norm(poses[i+1][:2] - path[indices[i+1]:indices[i+2]], axis=1) - radiuses[i+1])) + indices[i+1]

        plt.scatter(path[curve_index, 0], path[curve_index, 1], color='r', zorder=5)
        plt.scatter(path[curve_end_index, 0], path[curve_end_index, 1], color='r', zorder=5)
        plt.scatter(path[before_curve_index, 0], path[before_curve_index, 1], color='b', zorder=5)

        circ_x = poses[i+1][0] + radiuses[i+1] * np.cos(np.linspace(0, 2 * np.pi, num=100))
        circ_y = poses[i+1][1] + radiuses[i+1] * np.sin(np.linspace(0, 2 * np.pi, num=100))

        expanded_circ_x = poses[i+1][0] + radiuses[i+1] * np.cos(np.linspace(0, 2 * np.pi, num=100)) * 2.5
        expanded_circ_y = poses[i+1][1] + radiuses[i+1] * np.sin(np.linspace(0, 2 * np.pi, num=100)) * 2.5

        plt.plot(circ_x, circ_y, 'r--')
        plt.plot(expanded_circ_x, expanded_circ_y, 'r--')

    map = [
        [[-3.443, -3.462], [-0.019, -3.462]],
        [[-0.019, -3.462], [-0.019, 3.462]],
        [[-0.019, 3.462], [-3.443, 3.462]],
        [[-3.443, 3.462], [-3.443, -3.462]],
        [[-2.424, -3.462], [-2.424, -2.462]],
        [[-2.643, -1.319], [-0.019, -1.319]],
        [[-3.443, -0.081], [-2.157, -0.081]],
        [[-2.643, 1.157], [-1.338, 1.157]],
        [[-1.338, -1.319], [-1.338, 2.462]],
        [[-2.424, 2.462], [-2.424, 3.462]],
    ]
    if field_side == 'right':
        for i in range(len(map)):
            map[i][0][0] *= -1
            map[i][1][0] *= -1

    for line in map:
        start_point, end_point = line
        plt.plot([start_point[0], end_point[0]], [start_point[1], end_point[1]], 'k-')

    for index, i in zip(indices, range(len(indices))):
        draw_square(plt.gca(), path[index], 0.650, angles[index], color='b')
        if index != indices[-1]:
            inter_index = index + resolution / 2 if resolution % 2 == 0 else index + resolution // 2 # resolutionが偶数の場合と奇数の場合で処理を分ける
            draw_square(plt.gca(), path[int(inter_index)], 0.650, angles[int(inter_index)], color='r')
            if distance(path[index], path[indices[i+1]]) > 1.0:
                print(f"distance between {index} and {indices[i+1]} is {distance(path[index], path[indices[i+1]])}")
                inter_index2 = index + resolution / 3 if resolution % 3 == 0 else index + resolution // 3 # resolutionが3の倍数の場合とそうでない場合で処理を分ける
                inter_index3 = index + resolution / 3 * 2 if resolution % 3 == 0 else index + resolution // 3 * 2
                draw_square(plt.gca(), path[int(inter_index2)], 0.650, angles[int(inter_index2)], color='r')
                draw_square(plt.gca(), path[int(inter_index3)], 0.650, angles[int(inter_index3)], color='r')

    plt.title('Path with Interpolated Angles')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.legend()
    plt.grid(True)
    plt.show()

    # plt.plot(angles, label='Interpolated Angles')
    plt.plot(path[:, 0], label='X')
    plt.plot(path[:, 1], label='Y')
    plt.plot(speeds, label='Speeds')
    plt.plot(lookahead_distances, label='Lookahead Distances')
    plt.plot(p_gains, label='P Gains')
    plt.plot(i_gains, label='I Gains')
    plt.plot(path_p_gains, label='Path P Gains')
    plt.plot(path_i_gains, label='Path I Gains')
    plt.xlabel('Index')
    plt.ylabel('Value')
    plt.legend()