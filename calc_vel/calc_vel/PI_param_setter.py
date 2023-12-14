import yaml
import PySimpleGUI as sg

# PySimpleGUIのインターフェース設定
layout = [
    [sg.Text('P Gain'), sg.Slider(range=(0, 100), orientation='h', size=(15, 20), key='P_GAIN')],
    [sg.Text('I Gain'), sg.Slider(range=(0, 100), orientation='h', size=(15, 20), key='I_GAIN')],
    [sg.Text('Max Velocity'), sg.Slider(range=(0, 200), default_value=120, orientation='h', size=(15, 20), key='MAX_VEL')], 
    [sg.Button('Apply'), sg.Button('Exit')]
]

window = sg.Window('PID Controller Settings', layout)

while True:
    event, values = window.read()
    if event == sg.WIN_CLOSED or event == 'Exit':
        break
    elif event == 'Apply':
        updated_params = {
            'calc_vel': {
                'ros__parameters': {
                    'p_gain': float(values['P_GAIN']),
                    'i_gain': float(values['I_GAIN']),
                    'max_vel': float(values['MAX_VEL'])
                }
            }
        }
        with open('../yaml/pi_params.yaml', 'w') as file:
            yaml.dump(updated_params, file)
        print('P Gain: ', values['P_GAIN'])
        print('I Gain: ', values['I_GAIN'])

window.close()