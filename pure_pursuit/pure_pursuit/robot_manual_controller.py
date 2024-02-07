#!/usr/bin/env python3
import PySimpleGUI as sg
import numpy as np
import threading

speed = 1.0 # [m/s]
angular_speed = 0.5 # [rad/s]

class VelocityController:
    def __init__(self):
        self.vx = 0
        self.vy = 0
        self.omega = 0
        self.lock = threading.Lock()

    def update_velocity(self, key):
        with self.lock:
            if key == 'w:25':
                self.vy = speed
            elif key == 's:39':
                self.vy = -speed
            elif key == 'a:38':
                self.vx = -speed
            elif key == 'd:40':
                self.vx = speed
            elif key == 'e:26':
                self.omega = -angular_speed
            elif key == 'q:24':
                self.omega = angular_speed

    def reset_velocity(self):
        with self.lock:
            self.vx = 0
            self.vy = 0
            self.omega = 0

def draw_vector(graph, controller):
    while True:
        with controller.lock:
            vx, vy, omega = controller.vx, controller.vy, controller.omega

        graph.erase()
        graph.draw_line((0, 0), (vx / speed * 50, vy / speed * 50), color='blue', width=2)
        if omega > 0:
            graph.draw_line((50 * np.cos(np.pi / 180 * 30), 50 * np.sin(np.pi / 180 * 30)), (50 * np.cos(np.pi / 180 * 30) + omega / angular_speed * 50 * np.cos(np.pi / 180 * 120), 50 * np.sin(np.pi * 30 / 180) + omega / angular_speed * 50 * np.sin(np.pi / 180 * 120)), color='red', width=2)
            graph.draw_line((50 * np.cos(np.pi / 180 * 150), 50 * np.sin(np.pi / 180 * 150)), (50 * np.cos(np.pi / 180 * 150) + omega / angular_speed * 50 * np.cos(np.pi / 180 * 240), 50 * np.sin(np.pi / 180 * 150) + omega / angular_speed * 50 * np.sin(np.pi / 180 * 240)), color='red', width=2)
            graph.draw_line((50 * np.cos(np.pi / 180 * 270), 50 * np.sin(np.pi / 180 * 270)), (50 * np.cos(np.pi / 180 * 270) + omega / angular_speed * 50, 50 * np.sin(np.pi / 180 * 270)), color='red', width=2)
        elif omega < 0:
            graph.draw_line((50 * np.cos(np.pi / 180 * 30), 50 * np.sin(np.pi / 180 * 30)), (50 * np.cos(np.pi / 180 * 30) - omega / angular_speed * 50 * np.cos(np.pi / 180 * 120), 50 * np.sin(np.pi * 30 / 180) - omega / angular_speed * 50 * np.sin(np.pi / 180 * 120)), color='blue', width=2)
            graph.draw_line((50 * np.cos(np.pi / 180 * 150), 50 * np.sin(np.pi / 180 * 150)), (50 * np.cos(np.pi / 180 * 150) - omega / angular_speed * 50 * np.cos(np.pi / 180 * 240), 50 * np.sin(np.pi / 180 * 150) - omega / angular_speed * 50 * np.sin(np.pi / 180 * 240)), color='blue', width=2)
            graph.draw_line((50 * np.cos(np.pi / 180 * 270), 50 * np.sin(np.pi / 180 * 270)), (50 * np.cos(np.pi / 180 * 270) - omega / angular_speed * 50, 50 * np.sin(np.pi / 180 * 270)), color='blue', width=2)
        threading.Event().wait(0.01)  # 少し待機

def parameter_window():
    layout = [
        [sg.Text('Velocity:'), sg.Slider(range=(0, 1), default_value=0.5, orientation='h', key='-SPEED-', resolution=0.01)],
        [sg.Text('Angular Velocity:'), sg.Slider(range=(0, 1), default_value=0.5, orientation='h', key='-ANGULAR_SPEED-', resolution=0.01)],
        [sg.Button('OK')]
    ]

    window = sg.Window('Parameters', layout)
    return window

def update_parameters(window, controller):
    while True:
        event, values = window.read(timeout=10)
        if event == sg.WIN_CLOSED or event == 'OK':
            break
        if event == '-SPEED-':
            global speed
            speed = float(values['-SPEED-'])
        if event == '-ANGULAR_SPEED-':
            global angular_speed
            angular_speed = float(values['-ANGULAR_SPEED-'])

    window.close()

def create_gui():
    layout = [
        [sg.Graph(canvas_size=(400, 400), graph_bottom_left=(-100, -100), graph_top_right=(100, 100), background_color='white', key='-GRAPH-')],
        [sg.Text('Press W/A/S/D/E/Q to control velocity, R to reset')],
        [sg.Button('Exit')]
    ]

    window = sg.Window('Velocity Controller', layout, return_keyboard_events=True)
    return window

if __name__ == '__main__':
    controller = VelocityController()
    window = create_gui()

    # ベクトル描画スレッドを開始
    draw_thread = threading.Thread(target=draw_vector, args=(window['-GRAPH-'], controller), daemon=True)
    draw_thread.start()

    # パラメータ設定ウィンドウを開く
    param_thread = threading.Thread(target=update_parameters, args=(parameter_window(), controller), daemon=True)
    param_thread.start()

    # メインループ
    while True:
        event, values = window.read(timeout=10)
        if event == sg.WIN_CLOSED or event == 'Exit':
            break
        if event in ('w:25', 'a:38', 's:39', 'd:40', 'e:26', 'q:24'):
            controller.update_velocity(event)
        if event == 'r:27':
            controller.reset_velocity()

    window.close()
