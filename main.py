import ctypes
import serial
import matplotlib.pyplot as plt
import time
import keyboard
import struct
import pandas as pd


class ForcePID():
    Kp = 0  # 比例ゲイン
    Ki = 0  # 積分ゲイン
    Kd = 0  # 微分ゲイン

    epsilon = 0  # 目標値と現在値の差
    old_epsilon = 0  # 過去の目標値と現在値の差
    epsilon_sum = 0  # 目標値と現在値の差の積算

    min_position = 0
    max_position = 4095

    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    # 出力値を計算
    def output(self, force_target, force_now, now_position):
        self.epsilon = force_target - force_now
        self.epsilon_sum += self.epsilon
        print('epsilon:', self.epsilon)
        print('old_elpsilon', self.old_epsilon)
        print('epsilon_sum', self.epsilon_sum)
        print()
        output = now_position + self.Kp * (self.epsilon 
                            + self.Ki * self.epsilon_sum 
                            + self.Kd * (self.epsilon - self.old_epsilon))
        self.old_epsilon = self.epsilon

        if output > self.max_position:
            output = self.max_position
        elif output < self.min_position:
            output = self.min_position
        
        output = int(output)
        return output


def calc_zeros(dll, port_num, limit, status):
    print('wait for calc zeros')
    time.sleep(3)
    force_data = (ctypes.c_double * 6)()
    zeros = [0, 0, 0, 0, 0, 0]  # 0Nになる出力値
    repeat_time = 1000

    i = 0
    while i < repeat_time + 1000:
        if dll.GetSerialData(port_num, force_data, ctypes.pointer(status)) == True:
            print(i, force_data[2] / 1000 * limit[2])
            if i > 1000:
                for j in range(len(limit)):
                    zeros[j] += force_data[j] / 1000.0 * limit[j]
            i += 1

    for j in range(len(limit)):
        zeros[j] = zeros[j] / repeat_time
    print('in cale_zeros', zeros)
    return zeros


def main():
    Kp = 1
    Ki = 0
    Kd = 0
    
    port_num = 3  # センサのポート番号
    status = ctypes.c_char()
    serial_num = (ctypes.c_char * 9)()  # シリアルナンバー
    limit =(ctypes.c_double * 6)()  # センサ定格
    data = (ctypes.c_double * 6)()  # センサ出力
    data_newton = (ctypes.c_double * 6)()  # 力[N]
    zeros = (ctypes.c_double * 6)  # 0Nがセンサに印加されたときの力[N]の値
    data_name = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']  # dataに格納されている値

    com_motor = 'COM4'  # モータコントローラが接続されるCOMポート
    bit_rate = 9600  # シリアル通信のビットレート
    #ser = serial.Serial(com_motor, bit_rate, timeout = 0.01)


    plt.figure()

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
        if dll.SetSerialMode(port_num, True) == False:
            print('連続読み込みモードを開始できません')
            exit()

        #target = input('input target force[N]')
        PID = ForcePID(Kp, Ki, Kd)  # PIDインスタンス生成
        count = 0
        time_list = []
        Fz_list = []
        start_time = time.perf_counter()
        zeros = calc_zeros(dll, port_num, limit, status)
        #zeros = [0, 0, 0, 0, 0, 0]
        for i in range(len(zeros)):
            print(data_name[i], zeros[i])
    
        while True: 
            if dll.GetSerialData(port_num, data, ctypes.pointer(status)) == True:                
                for i in range(len(limit)):
                    data_newton[i] = limit[i] / 1000.0 * data[i] - zeros[i]
                    #print(data_name[i], end = ' ')
                    #print(data[i])

                Fz = data_newton[2]
                #print(data[2])

                time_passed = time.perf_counter() - start_time
                time_list.append(time_passed)
                Fz_list.append(data_newton[2])
                #print(count, end=' ')
                #qprint(Fz)

                
                """
                plt.cla()
                plt.plot(time_list, Fz_list)
                plt.ylim(-50, 50)
                plt.xlim(time_passed - 2, time_passed)
                plt.text(time_passed - 0.6, 4.1, '')
                plt.grid()
                plt.pause(0.001)
                """         
                       
                count += 1
            
            if keyboard.is_pressed('q'):  # qを押して停止
                    break
            
        plt.plot(time_list, Fz_list)
        plt.xlabel('tieme [s]')
        plt.ylabel('force [N]')
        plt.grid()
        plt.pause(10)
            
    finally:
        #print(Fz_list)
        dll.SetSerialMode(port_num, False)
        dll.PortClose()
        dll.Finalize()
        del dll
        print('完了')


if __name__ == '__main__':
    main()