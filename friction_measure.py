'''
手動でモータを動かしたときの引張力を測定するプログラム
'''

import main
import ctypes
import time
import keyboard
import matplotlib.pyplot as plt
import pandas as pd


port_num = 3  # センサのポート番号
status = ctypes.c_char()
serial_num = (ctypes.c_char * 9)()  # シリアルナンバー
limit =(ctypes.c_double * 6)()  # センサ定格
data = (ctypes.c_double * 6)()  # センサ出力
data_newton = (ctypes.c_double * 6)()  # 力[N]
zeros = (ctypes.c_double * 6)  # 0Nがセンサに印加されたときの力[N]の値
data_name = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']  # dataに格納されている値

com_motor = 'COM7'  # モータコントローラが接続されるCOMポート
bit_rate = 9600  # シリアル通信のビットレート

# dll読み込み
dll = ctypes.cdll.LoadLibrary(r'C:\Users\Shota\work\start_dash\project_python\CfsUsb.dll')  
dll.Initialize()  # dll初期化

try:
    # ポートを開く
    if dll.PortOpen(port_num) == False:
        print('ポートが開けません')
        exit()

    # センサ定格を確認
    if dll.GetSensorLimit(port_num, limit) == False:
        print('センサー定格が確認できません')
        exit()
    
    print('定格')
    for i in range(len(limit)):
        print(data_name[i], end = ':')
        print(limit[i])

    # シリアルナンバー確認
    if dll.GetSensorInfo(port_num, serial_num) == False:
        print('シリアルナンバーが確認できません')
        exit()
    
    # 連続読み込みモード開始
    '''
    if dll.SetSerialMode(port_num, True) == False:
        print('連続読み込みモードを開始できません')
        exit()
    '''
    time_list = []
    Fz_list = []
    zeros = main.calc_sensor_zeros(dll, port_num, limit, status)
    start_time = time.perf_counter()
    while True: 
        if dll.GetLatestData(port_num, data, ctypes.pointer(status)) == True: 
            time_passed = time.perf_counter() - start_time
            for i in range(len(limit)):
                data_newton[i] = limit[i] / 10000.0 * data[i] - zeros[i]

            Fz = data_newton[data_name.index('Fz')]
            print(Fz)
            time_list.append(time_passed)
            Fz_list.append(Fz)

            if keyboard.is_pressed('q'):  # qを押して停止
                break

    plt.figure()   
    plt.plot(time_list, Fz_list)
    plt.xlabel('tieme [s]')
    plt.ylabel('force [N]')
    plt.grid()
    plt.pause(5)

    result = pd.DataFrame(zip(time_list, Fz_list), columns = ['Time [s]', 'Fz [N]'])
    result.to_csv('friction.csv')


finally:
    dll.PortClose()
    dll.Finalize()
    del dll
    print('完了')