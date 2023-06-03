import ctypes
import serial
import matplotlib.pyplot as plt
import time
import keyboard
import struct
import pandas as pd
import numpy as np
from scipy import signal


# 力PID制御用のクラス
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

        if output > self.max_position:
            output = self.max_position
        elif output < self.min_position:
            output = self.min_position
        
        output = int(output)
        return output


# センサの0点を測定，計算する関数
def calc_sensor_zeros(dll, port_num, limit, status):
    print('calclating zeros')
    # 連続読み込みモード開始
    if dll.SetSerialMode(port_num, True) == False:
            print('連続読み込みモードを開始できません')
            exit()

    force_data = (ctypes.c_double * 6)()  # 測定した力
    zeros = [0, 0, 0, 0, 0, 0]  # 0Nになる出力値
    repeat_time = 1000  # 繰り返し回数

    # 力を繰り返し取得
    i = 0
    while i < repeat_time:
        if dll.GetSerialData(port_num, force_data, ctypes.pointer(status)) == True:
            for j in range(len(limit)):
                zeros[j] += force_data[j] / 10000.0 * limit[j]
            i += 1

    # 力の平均を計算
    for j in range(len(limit)):
        zeros[j] = zeros[j] / repeat_time

    # 連続読み込みモードを停止
    if dll.SetSerialMode(port_num, False) == False:
        print('連続読み込みモードを停止できません')
        exit()

    return zeros


def main():
    # 力制御PIDゲイン
    Kp = 1.1 #1, 0.8
    Ki = 0.0009
    Kd = 0.4 #0.2, 0.8, 0.6
    
    port_num = 3  # センサのポート番号
    status = ctypes.c_char()  # センサ情報
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
        elif dll.GetSensorLimit(port_num, limit) == False:
            print('センサー定格が確認できません')
            exit()
        # シリアルナンバー確認
        elif dll.GetSensorInfo(port_num, serial_num) == False:
            print('シリアルナンバーが確認できません')
            exit()

        # ローパスフィルタ
        cutoff_freq = 1  # カットオフ周波数
        order = 1  # フィルタのオーダー
        nyquist_freq = 160 / 2  # サンプリング周波数 / 2
        normalized_cutoff = cutoff_freq / nyquist_freq  
        b, a = signal.butter(order, normalized_cutoff, btype = 'low', analog = False)  # バターワースフィルタ


        # モード選択
        control_mode = input('input "f" to control force\ninput "x" to control position\n')
        if control_mode == "f":
            target_force = float(input('input target force[N]: '))  # 目標引張力
            #control_interval_sec = 0.005
            speed_byte_list = [0, 0]
        elif control_mode == "x":
            target_volume = float(input('input target volume 0 ~ 12 mL: '))
            target_position = int((-2075 + 1265) / 12 * target_volume + 2075)
            speed = int(input('input speed 3 ~ 1023 by integer: '))
            if not(0 <= speed <= 1023):
                print('speed should be 3 ~ 1023')
                exit()
            speed_byte_list = list(struct.pack('<H', speed))
            #control_interval_sec = (1023 - speed) / 100 / 10
            #now_target_position = 2080
        else:
            print('input "f" or "x"')
            exit()
        
        PID = ForcePID(Kp, Ki, Kd)  # PIDインスタンス生成
        
        # データ保存用の変数宣言
        time_list = []  # 経過時間
        Fz_list = []  # 力測定値
        Fz_filtered_list = []  # フィルタ後の力
        target_position_list = []  # 位置目標値
        now_position = 0  # 現在の位置の目標値

        zeros = calc_sensor_zeros(dll, port_num, limit, status)  # センサキャリブレーション
        
        send_data = [0, 1, 0]  # 送信用データ
        motor_serial = serial.Serial(com_motor, bordrate, timeout = 0.1)  # シリアル通信開始
        time.sleep(5)

        # sキーを押して動作開始
        print('press "s" key to start')
        while True:
            if keyboard.is_pressed('s'):
                break

        start_time = time.perf_counter()  # 開始時間
        time_passed = 0  # 経過時間
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
                # センサデータ取得
                for i in range(len(limit)):  
                    data_newton[i] = limit[i] / 10000.0 * data[i] - zeros[i]

                # データをリストに格納
                Fz = data_newton[data_name.index('Fz')]
                Fz_filtered_list = signal.lfilter(b, a, Fz_list)
                time_list.append(time_passed)
                Fz_list.append(Fz)
                target_position_list.append(now_position)
                try:
                    Fz_filtered = Fz_filtered_list[-1]
                except IndexError:
                    Fz_filtered = 0
                
                # 出力計算
                if control_mode == 'f':
                    output = PID.output(target_force, Fz_filtered, now_position)
                elif control_mode == 'x':
                    output = target_position

                # 出力送信
                send_data = list(struct.pack('<H', output))
                #send_data = list(send_data)
                if control_mode == 'f':
                    send_data.insert(0, 0)
                elif control_mode == 'x':
                    send_data.insert(0, 1)
                send_data = send_data + speed_byte_list
                motor_serial.write(send_data)  
                time.sleep(0.001)
                
                # データ受信
                read_data_binary = motor_serial.read_all()
                #print(read_data_binary)
                read_data = struct.unpack('3B', read_data_binary)
                now_position = read_data[1] + read_data[2] * 256

                # 力と現在の目標位置表示
                print('{:.3g}'.format(Fz), end = ' N\t ')
                print('{:.3g}'.format(Fz_filtered), end = ' N\t ')
                print(now_position)
                      
                # インターバル(現時点では0.005秒が最適，ローパスは0.005秒で設計)
                time.sleep(0.005) # 0.01
                #time.sleep(control_interval_sec)
                if len(time_list) > 50 and max(Fz_filtered_list[-50:]) - 5 > Fz:
                    send_data = [2, 0, 0, 0, 0]
                    motor_serial.write(send_data)
                    time.sleep(0.1)
                    break
                
            if keyboard.is_pressed('q'):  # qを押して停止
                send_data = [2, 0, 0, 0, 0]
                motor_serial.write(send_data)
                time.sleep(0.1)
                break
            elif keyboard.is_pressed('s'):
                #target_force = float(input('input target force[N]'))
                pass

        # グラフ描画
        Fz_filtered_list = np.insert(Fz_filtered_list, 0, 0)
        plt.figure()   
        plt.plot(time_list, Fz_filtered_list)
        plt.ylim(0, 25)
        plt.xlabel('Time [s]')
        plt.ylabel('Force [N]')
        plt.grid()
        plt.savefig('result.png')
        plt.pause(5)

        # 結果csv保存
        speed_result_list = []
        for i in range(len(time_list) - 1):
            speed_result_list.append(- (target_position_list[i + 1] - target_position_list[i]) / (time_list[i + 1] - time_list[i]) / 13.235)
        #speed_result_list = signal.lfilter(b, a, speed_result_list)
        result = pd.DataFrame(zip(time_list, Fz_list, Fz_filtered_list, target_position_list, speed_result_list),
                               columns = ['Time [s]', 'Fz [N]', 'Fz filtered [N]','target position [-]', 'speed'])
        file_name = input('input file name: ')
        result.to_csv('result\\' + file_name + '.csv')   
        print('saving as csv completed')
            
    finally:  # 終了時の処理
        send_data = [2, 0, 0, 0, 0]
        motor_serial.write(send_data)
        time.sleep(0.01)
        motor_serial.close()
        dll.PortClose()
        dll.Finalize()
        del dll
        print('完了')


if __name__ == '__main__':
    main()