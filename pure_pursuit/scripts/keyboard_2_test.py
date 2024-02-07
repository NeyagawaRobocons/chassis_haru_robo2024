import PySimpleGUI as sg
import time

# 同時押しの判定に使用する時間の閾値（秒）
simultaneous_threshold = 0.1

def main():
    layout = [[sg.Text("キーボード入力テスト")], [sg.Button("終了")]]
    window = sg.Window("キーボード入力", layout, return_keyboard_events=True)

    key_states = {'w:25': False, 'a:38': False, 's:39': False, 'd:40': False}
    key_timestamps = {'w:25': None, 'a:38': None, 's:39': None, 'd:40': None}

    while True:
        event, _ = window.read(timeout=10)

        if event == sg.WINDOW_CLOSED or event == "終了":
            break

        if event in key_states:
            key_states[event] = True  # キー押下
            key_timestamps[event] = time.time()  # タイムスタンプ記録
        elif event.startswith("w:25") or event.startswith("a:38") or event.startswith("s:39") or event.startswith("d:40"):
            key_states[event.split(':')[0]] = False  # キー離上

        # 同時押しの判定
        if key_states['w:25'] and key_states['a:38']:
            # WとAが同時に押された場合の処理
            if key_timestamps['w:25'] is not None and key_timestamps['a:38'] is not None:
                time_difference = abs(key_timestamps['w:25'] - key_timestamps['a:38'])
                if time_difference <= simultaneous_threshold:
                    print("WとAが同時に押されました")

        # 他のキーの組み合わせについても同様にチェック

    window.close()

if __name__ == '__main__':
    main()
