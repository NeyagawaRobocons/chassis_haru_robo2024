#!/usr/bin/env python3
import PySimpleGUI as sg
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import rcl_interfaces
import yaml
from ament_index_python.packages import get_package_share_directory
import os

# ROS 2パッケージ名を設定
pure_pursuit_package = 'pure_pursuit'

# パッケージの共有ディレクトリを取得
pure_pursuit_directory = get_package_share_directory(pure_pursuit_package)

# YAMLファイルへのパスを組み立て
pure_pursuit_param_path = os.path.join(pure_pursuit_directory, 'pure_pursuit_params.yaml')
pi_controller_param_path = os.path.join(pure_pursuit_directory, 'pi_controller_params.yaml')

class ParamControlNode(Node):
    def __init__(self):
        super().__init__('param_control_node')

    def set_remote_parameter(self, name, value):
        # パラメータクライアントを作成
        parameter_client = self.create_client(rcl_interfaces.srv.SetParameters, f'{self.target_node_name}/set_parameters')

        # サービスが利用可能になるまで待機
        while not parameter_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{self.target_node_name}/set_parameters service not available, waiting again...')

        # パラメータ変更リクエストを作成
        param = Parameter(name, Parameter.Type.DOUBLE, value)
        param_request = rcl_interfaces.srv.SetParameters.Request()
        param_request.parameters.append(param.to_parameter_msg())

        # リクエストを送信
        future = parameter_client.call_async(param_request)
        return future

def load_params_from_yaml(file_path, node_name):
    with open(file_path, 'r') as file:
        params = yaml.load(file, Loader=yaml.FullLoader)
    return params[node_name]['ros__parameters']

# YAMLファイルからパラメータを読み込む
pure_pursuit_params = load_params_from_yaml(pure_pursuit_param_path, 'pure_pursuit_node')
pi_controller_params = load_params_from_yaml(pi_controller_param_path, 'angle_PI_controller')

def main():
    # ROS 2ノードの初期化
    rclpy.init(args=None)
    param_node = ParamControlNode()

    # UIレイアウトの定義
    layout = [
        # pure_pursuit_node用のパラメータコントロール
        [sg.Text('P Gain X'), sg.Slider(range=(0, 10), resolution=0.1, orientation='h', size=(34, 20), key='p_gain_x', default_value=pure_pursuit_params['p_gain_x'])],
        [sg.Text('Speed'), sg.Slider(range=(0, 10), resolution=0.5, orientation='h', size=(34, 20), key='speed', default_value=pure_pursuit_params['speed'])],
        [sg.Text('Lookahead Distance'), sg.Slider(range=(0.0, 2.0), resolution=0.1, orientation='h', size=(34, 20), key='lookahead_distance', default_value=pure_pursuit_params['lookahead_distance'])],
        [sg.Text('Gain P'), sg.Slider(range=(0, 10), resolution=0.1, orientation='h', size=(34, 20), key='gain_p', default_value=pure_pursuit_params['gain_p'])],

        # angle_PI_controller用のパラメータコントロール
        [sg.Text('Angle Gain P'), sg.Slider(range=(0, 2), resolution=0.01, orientation='h', size=(34, 20), key='angle_gain_p', default_value=pi_controller_params['angle_gain_p'])],
        [sg.Text('Angle Gain I'), sg.Slider(range=(0, 0.5), resolution=0.001, orientation='h', size=(34, 20), key='angle_gain_i', default_value=pi_controller_params['angle_gain_i'])],

        # Apply、Save、Exitボタン
        [sg.Button('Apply'), sg.Button('Save'), sg.Button('Exit')]
    ]

    window = sg.Window('PI Control Parameters', layout)
    
    # イベントループ
    while True:
        event, values = window.read()
        if event in (None, 'Exit'):
            break
        elif event == 'Apply':
            for param_name, param_value in values.items():
                future = param_node.set_remote_parameter(param_name, float(param_value))
                rclpy.spin_until_future_complete(param_node, future)
                result = future.result()
                if result is not None:
                    param_node.get_logger().info(f'Parameter {param_name} set to {param_value}')

        elif event == 'Save':
            # パラメータを適切な型に変換
            for key in ['speed', 'lookahead_distance', 'gain_p', 'angle_gain_p', 'angle_gain_i']:
                if key in values:
                    values[key] = float(values[key])  # 文字列を数値に変換
                if key in ['speed', 'lookahead_distance', 'gain_p']:
                    with open(pure_pursuit_param_path, 'w') as file:
                        yaml.dump({'pure_pursuit_node': {'ros__parameters': values}}, file)
                    param_node.get_logger().info('pure_pursuit params saved!')
                elif key in ['angle_gain_p', 'angle_gain_i']:
                    with open(pi_controller_param_path, 'w') as file:
                        yaml.dump({'angle_PI_controller': {'ros__parameters': values}}, file)
                    param_node.get_logger().info('angle_PI_controller params saved!')

    # ウィンドウを閉じる
    window.close()

    # ROS 2のシャットダウン
    param_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
