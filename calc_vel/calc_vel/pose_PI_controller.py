# description: 位置制御を行うノード
# input1: 目標位置：/goal_poseトピック(geometry_msgs/msg/PoseStamped型)
# input2: 現在位置：/robot_poseトピック(geometry_msgs/msg/Pose型)
# output: 速度指令：/input_velトピック(std_msgs/msg/Float64MultiArray型)
# 計算: PI_controller_class.pyのPI_controllerクラスを使用
# パラメータ: p_gain, i_gain -> yamlファイル(../yaml/pi_params.yaml)から読み込み
# 起動コマンド: 
# cd ~/ros2_ws/src/chassis_haru_robo2024/calc_vel/calc_vel
# ros2 run calc_vel calc_vel --ros-args --params-file ../yaml/pi_params.yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray as Float64
from .PI_controller_class import PI_controller
import PySimpleGUI as sg
import yaml

PI_controller = PI_controller()

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('calc_vel')
        self.vel_pub = self.create_publisher(
            Float64,
            'input_vel',
            10)

        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10)
        self.goal_sub  # 警告を回避するために設置されているだけです。削除しても挙動はかわりません。

        self.robot_sub = self.create_subscription(
            Pose,
            'robot_pose',
            self.robot_callback,
            10)
        self.robot_sub

        # パラメータを宣言
        self.declare_parameter('p_gain', 0.0)  # 0.0はデフォルト値
        self.declare_parameter('i_gain', 0.0)  # 同様にi_gainも宣言

        # パラメータの現在値を取得
        self.p_gain = self.get_parameter('p_gain').get_parameter_value().double_value
        self.i_gain = self.get_parameter('i_gain').get_parameter_value().double_value

        self.goal = Pose()
        self.Kp = 0.0
        self.Ki = 0.0

    def goal_callback(self, msg: Pose):
        # description: 目標位置を取得，格納
        self.goal = msg
        self.get_logger().info('I heard: "%s"' % self.goal)

    def robot_callback(self, msg: Pose):
        vels = Float64()
        vels.data = [0.0, 0.0, 0.0] # テスト用の値
        self.vel_pub.publish(vels)

    def update_gain(self, p_gain, i_gain):
        # パラメータを更新
        self.set_parameters([
            rclpy.parameter.Parameter('p_gain', rclpy.Parameter.Type.DOUBLE, p_gain),
            rclpy.parameter.Parameter('i_gain', rclpy.Parameter.Type.DOUBLE, i_gain)
        ])

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    # PySimpleGUIのインターフェース設定
    layout = [
        [sg.Text('P Gain'), sg.Slider(range=(0, 100), orientation='h', size=(15, 20), key='P_GAIN')],
        [sg.Text('I Gain'), sg.Slider(range=(0, 100), orientation='h', size=(15, 20), key='I_GAIN')],
        [sg.Button('Apply'), sg.Button('Save'), sg.Button('Exit')]
    ]

    window = sg.Window('PID Controller Settings', layout)

    while True:
        event, values = window.read()
        if event == sg.WIN_CLOSED or event == 'Exit':
            break
        elif event == 'Apply':
            p_gain = values['P_GAIN']
            i_gain = values['I_GAIN']
            minimal_subscriber.update_gain(p_gain, i_gain)
        elif event == 'Save':
            updated_params = {
                'calc_vel': {
                    'ros__parameters': {
                        'p_gain': values['P_GAIN'],
                        'i_gain': values['I_GAIN']
                    }
                }
            }
            with open('../yaml/pi_params.yaml', 'w') as file:
                yaml.dump(updated_params, file)

    window.close()

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()