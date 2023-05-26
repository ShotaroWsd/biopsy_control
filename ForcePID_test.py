"""
class FrocePIDのテストプログラム
"""

from main import ForcePID
import matplotlib.pyplot as plt 
import struct

def main():
    Kp = 100
    Ki = 0
    Kd = 0
    '''
    target = 10
    now = 5
    now_position = 100
    
    PID = ForcePID(Kp, Ki, Kd)
    for i in range(0, 4000, 100):
        output = PID.output(target, now, i)
        print('output', output)    
    return 0
    '''
    target_position = int(input('input target position 0~4095:'))
    send_data = [0, target_position, target_position >> 8]
    #recieve_data = bytes([0, target_position, target_position << 8])

    #recieve_data = struct.unpack('3B', recieve_data)
    data = bytes(target_position)
    print(data)
    print(send_data)
    #print(recieve_data)

if __name__ == '__main__':
    main()

