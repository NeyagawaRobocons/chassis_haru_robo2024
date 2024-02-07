import PySimpleGUI as sg

def main():
    # レイアウトの定義
    layout = [
        [sg.Text("キーボード入力をテストする", key="-OUTPUT-")],
        [sg.Button("終了")]
    ]

    # ウィンドウの作成
    window = sg.Window("キーボード入力テスト", layout, return_keyboard_events=True)

    # イベントループ
    while True:
        event, values = window.read()

        # ウィンドウを閉じるイベントが発生したら終了
        if event == sg.WINDOW_CLOSED or event == "終了":
            break

        print(event)
        
        # キーボードイベントの処理
        if event in ('w:25', 'a:38', 's:39', 'd:40', 'q:24', 'e:26'):
            window['-OUTPUT-'].update(f'キー "{event}" が押されました。')

    window.close()

if __name__ == '__main__':
    main()
