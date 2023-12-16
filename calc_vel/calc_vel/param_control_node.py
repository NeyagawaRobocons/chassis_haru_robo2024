import PySimpleGUI as sg
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import rcl_interfaces
import yaml
from ament_index_python.packages import get_package_share_directory
import os

# ROS 2パッケージ名
package_name = 'calc_vel'

# パッケージの共有ディレクトリを取得
package_share_directory = get_package_share_directory(package_name)

# YAMLファイルへのパスを組み立て
param_file_path = os.path.join(package_share_directory, 'pi_params.yaml')

class ParamControlNode(Node):
    def __init__(self):
        super().__init__('param_control_node')
       # ターゲットノードの名前を指定します。
        self.target_node_name = 'calc_vel'

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

def main():
    # ROS 2ノードの初期化
    rclpy.init(args=None)
    param_node = ParamControlNode()

    # UIレイアウトの定義
    layout = [
        [sg.Text('P Gain X'), sg.Slider(range=(0, 10), resolution=0.1, orientation='h', size=(34, 20), key='p_gain_x')],
        [sg.Text('I Gain X'), sg.Slider(range=(0, 10), resolution=0.1, orientation='h', size=(34, 20), key='i_gain_x')],
        [sg.Text('P Gain Y'), sg.Slider(range=(0, 10), resolution=0.1, orientation='h', size=(34, 20), key='p_gain_y')],
        [sg.Text('I Gain Y'), sg.Slider(range=(0, 10), resolution=0.1, orientation='h', size=(34, 20), key='i_gain_y')],
        [sg.Text('P Gain Theta'), sg.Slider(range=(0, 10), resolution=0.1, orientation='h', size=(34, 20), key='p_gain_theta')],
        [sg.Text('I Gain Theta'), sg.Slider(range=(0, 10), resolution=0.1, orientation='h', size=(34, 20), key='i_gain_theta')],
        [sg.Text('Max Input'), sg.InputText('30.0', key='max_input'), sg.Text('rad/s')],
        [sg.Text('Radius'), sg.InputText('0.051', key='radius'), sg.Text('m')],
        [sg.Text('Length'), sg.InputText('0.295', key='length'), sg.Text('m')],
        [sg.Button('Apply'), sg.Button('Save'), sg.Button('Exit')]
    ]

    # ウィンドウの作成
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
            for key in ['p_gain_x', 'i_gain_x', 'p_gain_y', 'i_gain_y', 'p_gain_theta', 'i_gain_theta', 'max_input', 'radius', 'length']:
                if key in values:
                    values[key] = float(values[key])  # 文字列を数値に変換
            # YAMLファイルへの保存処理をここに記述
            # ファイルを開く
            with open(param_file_path, 'w') as file:
                yaml.dump({'calc_vel': {'ros__parameters': values}}, file)
            param_node.get_logger().info('Saved!')

    # ウィンドウを閉じる
    window.close()

    # ROS 2のシャットダウン
    param_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()