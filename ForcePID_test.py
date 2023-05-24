"""
class FrocePIDのテストプログラム
"""

from main import ForcePID
import matplotlib.pyplot as plt 


def main():
    Kp = 100
    Ki = 0
    Kd = 0
    
    target = 10
    now = 5
    now_position = 100

    PID = ForcePID(Kp, Ki, Kd)
    for i in range(0, 4000, 100):
        output = PID.output(target, now, i)
        print('output', output)    
    return 0


if __name__ == '__main__':
    main()

