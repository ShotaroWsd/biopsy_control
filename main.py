import ctypes
import serial
import matplotlib.pyplot as plt
import time
import keyboard
import struct
import pandas as pd
import numpy as np
from scipy import signal

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
        output = now_position - self.Kp * (self.epsilon 
                            + self.Ki * self.epsilon_sum 
                            + self.Kd * (self.epsilon - self.old_epsilon))
        self.old_epsilon = self.epsilon

        if force_target - 1 < force_now < force_target + 1:
            #return int(now_position)
            pass

        elif output > self.max_position:
            output = self.max_position
        elif output < self.min_position:
            output = self.min_position
        elif np.abs(output - now_position) < 3 and np.abs(self.epsilon) < 0.05 * force_target :
            #output = now_position
            pass
        output = int(output)
        return output


def calc_sensor_zeros(dll, port_num, limit, status):
    print('wait for calc zeros')
    if dll.SetSerialMode(port_num, True) == False:
            print('連続読み込みモードを開始できません')
            exit()
    force_data = (ctypes.c_double * 6)()  
    zeros = [0, 0, 0, 0, 0, 0]  # 0Nになる出力値
    repeat_time = 1000

    i = 0
    while i < repeat_time:
        if dll.GetSerialData(port_num, force_data, ctypes.pointer(status)) == True:
            #print(i, force_data[2] / 1000 * limit[2])
            for j in range(len(limit)):
                zeros[j] += force_data[j] / 10000.0 * limit[j]
            i += 1

    for j in range(len(limit)):
        zeros[j] = zeros[j] / repeat_time

    if dll.SetSerialMode(port_num, False) == False:
        print('連続読み込みモードを停止できません')
        exit()
    return zeros


def main():
    # PIDゲイン
    Kp = 1.1 #1, 0.8
    Ki = 0.0009
    Kd = 0.4 #0.2, 0.8, 0.6
    
    port_num = 3  # センサのポート番号
    status = ctypes.c_char()
    serial_num = (ctypes.c_char * 9)()  # シリアルナンバー
    limit =(ctypes.c_double * 6)()  # センサ定格
    data = (ctypes.c_double * 6)()  # センサ出力
    data_newton = (ctypes.c_double * 6)()  # 力[N]
    zeros = (ctypes.c_double * 6)  # 0Nがセンサに印加されたときの力[N]の値
    data_name = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']  # dataに格納されている値

    com_motor = 'COM7'  # モータコントローラが接続されるCOMポート
    bordrate = 9600  # シリアル通信のビットレート
    
    # dll読み込み
    dll_path = r'C:\Users\Shota\work\start_dash\project_python\CfsUsb.dll'
    dll = ctypes.cdll.LoadLibrary(dll_path)  
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
        # シリアルナンバー確認
        if dll.GetSensorInfo(port_num, serial_num) == False:
            print('シリアルナンバーが確認できません')
            exit()


        # low pass filter
        cutoff_freq = 1
        order = 1

        nyquist_freq = 160 / 2
        normalized_cutoff = cutoff_freq / nyquist_freq
        b, a = signal.butter(order, normalized_cutoff, btype = 'low', analog = False)


        send_data = [0, 1, 0]
        target_force = float(input('input target force[N]'))
        PID = ForcePID(Kp, Ki, Kd)  # PIDインスタンス生成
        time_list = []
        Fz_list = []
        Fz_filtered_list = []
        target_position_list = []
        now_position = 0
        zeros = calc_sensor_zeros(dll, port_num, limit, status)
        motor_serial = serial.Serial(com_motor, bordrate, timeout = 0.1)
        time.sleep(5)

        print('press "s" to start')
        while True:
            if keyboard.is_pressed('s'):
                break
        start_time = time.perf_counter()
        '''
        target_position = int(input('input target position 0~4095:'))
        send_data = [0, target_position, target_position >> 8]
        send_data = struct.pack('<H', target_position)
        send_data = list(send_data)
        send_data.insert(0, 0)
        print(send_data)
        '''    
        while True: 
            if dll.GetLatestData(port_num, data, ctypes.pointer(status)) == True: 
                time_passed = time.perf_counter() - start_time
                for i in range(len(limit)):
                    data_newton[i] = limit[i] / 10000.0 * data[i] - zeros[i]

                Fz = data_newton[data_name.index('Fz')]
                Fz_filtered_list = signal.lfilter(b, a, Fz_list)
                #print(type(Fz_filtered_list))
                try:
                    Fz_filtered = Fz_filtered_list[-1]
                except IndexError:
                    Fz_filtered = 0
                    #Fz_filtered_list = np.append(Fz_filtered_list, 0)

                output = PID.output(target_force, Fz_filtered, now_position)
                send_data = struct.pack('<H', output)
                send_data = list(send_data)
                send_data.insert(0, 0)
                #print(send_data)

                motor_serial.write(send_data)  
                time.sleep(0.001)
                print(round(Fz_filtered, 3), end = ' N ')
                time_list.append(time_passed)
                Fz_list.append(Fz)
                
                read_data_binary = motor_serial.read_all()
                #print(read_data_binary, end = ':')
                read_data = struct.unpack('3B', read_data_binary)
                now_position = read_data[1] + read_data[2] * 256
                print(now_position)
                target_position_list.append(now_position)
                
                
                """
                plt.cla()
                plt.plot(time_list, Fz_list)
                plt.ylim(-50, 50)
                plt.xlim(time_passed - 2, time_passed)
                plt.text(time_passed - 0.6, 4.1, '')
                plt.grid()
                plt.pause(0.001)
                """         
                # 現時点では0.005秒が最適
                time.sleep(0.005) # 0.01

            if keyboard.is_pressed('q'):  # qを押して停止
                send_data = [1, 0, 0]
                motor_serial.write(send_data)
                break
            elif keyboard.is_pressed('s'):
                target_force = float(input('input target force[N]'))

        Fz_filtered_list = np.insert(Fz_filtered_list, 0, 0)
        plt.figure()   
        plt.plot(time_list, Fz_filtered_list)
        plt.ylim(0, 30)
        plt.xlabel('Time [s]')
        plt.ylabel('Force [N]')
        plt.grid()
        plt.savefig('result.png')
        plt.pause(5)
        
        result = pd.DataFrame(zip(time_list, Fz_list, Fz_filtered_list, target_position_list),
                               columns = ['Time [s]', 'Fz [N]', 'Fz filtered [N]','target position [-]'])
        result.to_csv('result.csv')   
            
    finally:
        #print(Fz_list)
        motor_serial.close()
        #dll.SetSerialMode(port_num, False)
        dll.PortClose()
        dll.Finalize()
        del dll
        print('完了')


if __name__ == '__main__':
    main()