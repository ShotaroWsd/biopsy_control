from ctypes import *
import serial
import matplotlib.pyplot as plt
import time
import keyboard

def main():
    #ser = serial.Serial('COM3', 9600, timeout = 0.01)
    global port_num
    port_num = 3  # センサのポート番号
    status = c_char()
    serial_num = (c_char * 9)()  # シリアルナンバー
    limit =(c_double * 6)()  # センサ定格
    data = (c_double * 6)()  # センサ出力
    data_newton = (c_double * 6)()  # 力[N]
    data_name = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']

    plt.figure()

    global dll
    dll = cdll.LoadLibrary(r'C:\Users\Shota\work\start_dash\project_python\CfsUsb.dll')  # dll読み込み
    dll.Initialize()  # dll初期化

    # ポートを開く
    if dll.PortOpen(port_num) == False:
        print('ポートが開けません')
        exit()

    # センサ定格を確認
    if dll.GetSensorLimit(port_num, limit) == False:
        print('センサー定格が確認できません')
        exit()
    
    for i in range(len(limit)):
        print(limit[i])

    # シリアルナンバー確認
    if dll.GetSensorInfo(port_num, serial_num) == False:
        print('シリアルナンバーが確認できません')
        exit()
    
    # 連続読み込みモード開始
    if dll.SetSerialMode(port_num, True) == False:
        print('連続読み込みモードを開始できません')
        exit()

    count = 0
    time_list = []
    Fz_list = []
    start_time = time.perf_counter()

    while count < 10000: 
        if dll.GetSerialData(port_num, data, pointer(status)) == True:
            
            for i in range(len(limit)):
                data_newton[i] = limit[i] / 10000 * data[i]
                #print(data_name[i], end = ' ')
                #print(data[i])

            Fz = limit[2] / 10000.0 * data[2]
            #print(data[2])

            time_passed = time.perf_counter() - start_time
            time_list.append(time_passed)
            Fz_list.append(Fz)
            #time.sleep(0.01)
            print(Fz)
            #print(Fz_list[count])
            
            '''
            plt.cla()
            plt.plot(time_list, Fz_list)
            plt.ylim(-50, 50)
            plt.xlim(time_passed - 2, time_passed)
            plt.text(time_passed - 0.6, 4.1, '')
            plt.grid()
            plt.pause(0.001)

            time.sleep(0.1)
            '''
            
            count += 1

if __name__ == '__main__':
    try:
        main()
    finally:
        dll.SetSerialMode(port_num, False)
        dll.PortClose()
        dll.Finalize()
        #del dll
        print('完了')



    