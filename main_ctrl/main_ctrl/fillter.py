import numpy as np
import math 

def comp_filter(alpha, acc, gyro, euler):
    dt = 0.01

    ax = acc[0]
    ay = acc[1]
    az = acc[2]

    gx = gyro[0]
    gy = gyro[1]
    gz = gyro[2]
    
    Lpf = LowPassFilter(10,0.001)
    ax_filter  = Lpf.filter(ax)
    roll, pitch, yaw = euler
    
    
    f= open("ax.txt","w+")
    f.write("%d %d" % (ax, ax_filter))
    print(111111111)
    roll_acc = math.atan(-ay/az)
    pitch_acc = math.atan(-ax / math.sqrt(ay**2 + az**2))

    roll_gyro = roll + (gx * dt) # deg/s
    pitch_gyro = pitch + (gy * dt)
    yaw_gyro = yaw + (gz * dt)

    roll = alpha * roll_gyro + (1-alpha) * roll_acc
    pitch = alpha * pitch_gyro + (1-alpha) * pitch_acc
    yaw = alpha * yaw_gyro
    roll = 1
    
    return roll, pitch, yaw



class LowPassFilter(object):
    def __init__(self, cut_off_freqency, ts):
    	# cut_off_freqency: 차단 주파수
        # ts: 주기
        
        self.ts = ts
        self.cut_off_freqency = cut_off_freqency
        self.tau = self.get_tau()

        self.prev_data = 0.
        
    def get_tau(self):
        return 1 / (2 * np.pi * self.cut_off_freqency)

    def filter(self, data):
        val = (self.ts * data + self.tau * self.prev_data) / (self.tau + self.ts)
        self.prev_data = val
        return val
    
    