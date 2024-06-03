# Copyright 2016 Open Source Robotics Foundation, Inc.
# 3
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from __future__ import division
import time
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import time
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy

# servo control
imu_threshold = -0.2
front_threshold = 30 #나중에 꺾을 때 앞에 초음파 z

# align()
threshold = 0.5 # Align threshold
K_align = 0.1 # control gain
V_align = 0.5# speed while aligning

# Climb()
v_climb = 200
w_climb = 0
threshold_climb = 0.2

# turn()
v_turn = 0
w_turn = 0.18
threshold_turn = 0.1

v_align = 0
w_align = 0.18

# PCA9685모듈을 임포트.
import Adafruit_PCA9685
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)
# pwm = Adafruit_PCA9685.PCA9685(address=0x70, busnum=4)
#from adafruit circuit python
# import board
# from adafruit_motor import servo
# from adafruit_pca9685 import PCA9685

# i2c = board.I2C()  # uses board.SCL and board.SDA
# pca = PCA9685(i2c)
# pca.frequency = 50


bldc_L_ch = 3
bldc_R_ch = 1
servo_ch = 2
#  set servo pulse width min, mid, and max
servo_up =  220  #ulse length out of 4096
servo_mid    = 285 # Middle pulse length out of 4096
servo_down   = 330

 # pulse length out of 4096

# by Encoder Team
# for PCA9685, it use 12 bit resolution
# min mulse of esc is 1ms, max is 2ms
# 20ms is one period of 50Hz
# min = 1ms/20ms x 4096(12 bit) = 204.8 ~= 205
# mid = 307
# max = 2ms/20ms x 4096(12 bit) = 409.6 ~= 409

#### in our robot, motor stop at 317 bit. so added min max += 10
# set min, mid, max of motor pwm
motor_pwm_min_L = 215
motor_pwm_mid_L = 317
motor_pwm_max_L = 419

motor_pwm_min_R = 317 - 30
motor_pwm_mid_R = 317
motor_pwm_max_R = 317 + 30

motor_pwm_mid = 317

# simplify setting servo pulse
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    # pulse_length //= 60       # 60 Hz
    pulse_length //= 50       # 50 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)
# 서보모터(SG90)에 최적화된 69Hz로 펄스주기를 설정.


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription_sensor = self.create_subscription(
            Imu,
            'imu',
            self.sensor_callback,
            10)
        self.subscription_sensor # prevent unused variable warning
    
        self.subscription_joy = self.create_subscription(
            Joy,
            'joy',
            self.joystick_callback,
            10)
        self.subscription_joy  # prevent unused variable warning

        self.timer = self.create_timer(0.005, self.actuate)

        self.state = 1
        self.substate = 0
        self.trial = 0
                
        self.euler = [0, 0, 0] # [roll, pitch, yaw]
        self.distance = [0.0, 0.0, 0.0] # [left, right, Front]
        
        self.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        self.servo = servo_up
        self.yaw = None  # for turning
        self.motor_cmd = [0.0, 0.0, 0.0, 0.0] #yaw velocity,speed (0, 1 : left) (2, 3 : right)
        self.prev_left_pwm = 317
        self.prev_right_pwm = 317
        self.w_z = 0.0


        # self.check_pwm_range = 317
        self.previous_time = time.time()
        self.previous_v = [317,317] # using like speed buffer in motor control
        self.previous_t = 0.0 # using like time buffer in motor control
    
        self.button_B_rising_edge = False # toggle state 0 and 1
        self.button_A_rising_edge = False # enter substate 1 (align)
        self.button_X_rising_edge = False # enter substate 2 (climb)
        self.button_Y_rising_edge = False # enter substate 3 (turn)
                               
    def sensor_callback(self, msg):

        #self.get_logger().info('I heard: "%d"' % msg.angular_velocity_covariance[0])
        self.euler[0] = msg.linear_acceleration_covariance[0]   # roll
        self.euler[1] = msg.linear_acceleration_covariance[4]  #pitch
        self.euler[2] = msg.linear_acceleration_covariance[8]  #yaw
        self.distance[0] = msg.angular_velocity_covariance[0] # LF sonar distance
        self.distance[1] = msg.angular_velocity_covariance[4] # RF sonar distance
        self.distance[2] = msg.angular_velocity_covariance[8] # Front sonar distance
        self.w_z = msg.linear_acceleration_covariance[1] # z angular velocity
        print(self.euler)
        self.servo_control()
        
        
# (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def joystick_callback(self, msg):
        self.axes = msg.axes
        self.buttons = msg.buttons
                    
        if (self.buttons[2] == 1 and self.button_B_rising_edge == False):
            self.button_B_rising_edge = True
            self.state = not self.state

        if self.buttons[2] == 0:
            self.button_B_rising_edge = False

        # button A (substate 1, align)
        if self.buttons[1] == 1:
            self.substate = 1
        # button X (substate 2, climb)
        elif self.buttons[0] == 1:
            self.substate = 2
        # button Y (substate 3, turn)
        elif self.buttons[3] == 1:
            self.substate = 3



        self.motor_cmd[1] = msg.axes[1] #1
        self.motor_cmd[0] = msg.axes[0] #0
        self.motor_cmd[2] = msg.axes[2] #1
        self.motor_cmd[3] = msg.axes[3] #0


        if self.axes[4] == 1 :
            self.servo += 5
        if self.axes[4] == -1 :
            self.servo -= 5

        self.actuate()
        
    def actuate(self):
    
        #if self.state == 0:
            #print(self.state)

        #elif self.state == 1:
        if self.substate == 1:
            self.cruise()
        elif self.substate == 2:
            self.motor_cmd = [0.0, 0.0,0.0,0.0]
            self.substate = 0
        elif self.substate == 3:
            self.turn()

        self.motor_control()

    def motor_control(self): 
        # # Mapping v and w to left and right motor pwm signal
        if self.motor_cmd[2] != 0 or self.motor_cmd[3] != 0:
            motor_pwm_min = motor_pwm_min_R
            motor_pwm_max = motor_pwm_max_R
            v = self.motor_cmd[3]
            w = self.motor_cmd[2]
            w_weight = 0.7071
            v_weight = np.sqrt(1 - w_weight**2)
            scale = 1
        else :
            motor_pwm_min = motor_pwm_min_L
            motor_pwm_max = motor_pwm_max_L
            v = self.motor_cmd[1]
            w = self.motor_cmd[0]
            w = 0 # for pwm tuning
            w_weight = 0.1
            v_weight = np.sqrt(1 - w_weight**2)
            scale = 0.865
        # turn_treshold = 1 #1
        # # acc = (v-self.previous_v)/(current_time - self.previous_time)
        # # print("Acceleration: ", acc, v*w)

        # # if acc > accel_treshold_forward:
        # #     v = self.previous_v + (current_time - self.previous_time)*accel_treshold_forward
        # # elif acc < accel_treshold_backward:
        # #     v = self.previous_v + (current_time - self.previous_time)*accel_treshold_backward

        # # if v*w > turn_treshold and v > 0:
        # #     w = turn_treshold/v
        # self.previous_v = v
        # # d = wid
        # d = 0.5
        
        # V_left = (np.clip((v-(w*d/2)),-1, 1)) # clip if V is bigger then maximum velocity.
        # V_right = (np.clip((v+(w*d/2)),-1, 1))
        V_left = v_weight*v - w_weight * w
        V_right = v_weight*v + w_weight * w
        V_right = scale * V_right
        if self.buttons[4] == 1:
            V_left = -0.15
            V_right = 0.15
        if self.buttons[5] == 1:
            V_left = 0.15
            V_right = -0.15

        if V_left < 0:
            V_left_mapped = np.interp(V_left, [-1, 0], [motor_pwm_min, motor_pwm_mid]) # change 180 to pwm min max                    
        else:
            V_left_mapped = np.interp(V_left, [0, 1], [motor_pwm_mid, motor_pwm_max]) # change 180 to pwm min max                    

        if V_right < 0:
            V_right_mapped = np.interp(V_right, [-1, 0], [motor_pwm_min, motor_pwm_mid]) # change 180 to pwm min max                    
        else:
            V_right_mapped = np.interp(V_right, [0, 1], [motor_pwm_mid, motor_pwm_max]) # change 180 to pwm min max                    

                
        accel_treshold_forward =  1  #0.3 #1
        accel_treshold_backward = -1 #-0.8 #1
        current_time = time.time()

        acc_L = V_left_mapped -self.previous_v[0]
        if acc_L > accel_treshold_forward:
            V_left_mapped  = self.previous_v[0] + accel_treshold_forward
        elif acc_L < accel_treshold_backward:
            V_left_mapped  = self.previous_v[0] + accel_treshold_backward




        acc_R = V_right_mapped -self.previous_v[1]
        if acc_R > accel_treshold_forward:
            V_right_mapped  = self.previous_v[1] + accel_treshold_forward
        elif acc_R < accel_treshold_backward:
            V_right_mapped  = self.previous_v[1] + accel_treshold_backward

 
        if self.buttons[2] == 1:
            V_left_mapped = 317
            V_right_mapped = 317        

        self.previous_v = [V_left_mapped, V_right_mapped]

        # print("check motor pwm V_left, V_right,servo,  freq: ", 
        #         int(V_left_mapped), int(V_right_mapped),int(self.servo), 1/(current_time - self.previous_time))
        self.get_logger().info("check motor pwm V_left, V_right, substate, freq, v: {}, {}, {}, {},{} ".format( 
                int(V_left_mapped), int(V_right_mapped),int(self.substate), 1/(current_time - self.previous_time), self.w_z
                ))

        pwm.set_pwm(bldc_L_ch, 0, int(V_left_mapped))
        pwm.set_pwm(bldc_R_ch, 0, int(V_right_mapped))
        pwm.set_pwm(servo_ch, 0, self.servo)

        ## for pwm test_start
        # current_time = time.time()
        # self.prev_left_pwm += self.axes[4]
        # self.prev_right_pwm += self.axes[5]

        # V_left_mapped = self.prev_left_pwm
        # V_right_mapped = self.prev_right_pwm

        # pwm.set_pwm(bldc_L_ch, 0, int(V_left_mapped))
        # pwm.set_pwm(bldc_R_ch, 0, int(V_right_mapped))
        # pwm.set_pwm(servo_ch, 0, self.servo)
        # self.get_logger().info("check motor pwm V_left, V_right, substate, freq: {}, {}, {}, {} ".format( 
        #         int(V_left_mapped), int(V_right_mapped),int(self.substate), 1/(current_time - self.previous_time)
        #         ))
        ## for pwm test_end
        
        self.previous_time = current_time

        # self.get_logger().info("hihi")
        ###### uncomments
        # if self.axes[5] == 1 :
        #     self.check_pwm_range += 1
        # if self.axes[5] == -1 :
        #     self.check_pwm_range -= 1

        # pwm.set_pwm(bldc_L_ch, 0, int(self.check_pwm_range))
        # pwm.set_pwm(bldc_R_ch, 0, int(self.check_pwm_range))        
        # print("set motor   pwm V_left and V_right : ", int(self.check_pwm_range))
    

    
    def servo_control(self):
        if self.euler[1] < imu_threshold:
            if self.servo < servo_mid:
                #self.servo = self.servo + 10
                #if self.servo > servo_mid:
                self.servo = servo_mid
           
        else:
            if self.servo > servo_up:
                self.servo = self.servo -30 
                #self.servo = servo_up   
                if self.servo < servo_up:
                    self.servo = servo_up   
        print("Threshold", self.euler[1], self.distance[2])

        
        # if self.euler[1] < imu_threshold:
        #     if self.distance[2] > front_threshold:  # 0인건 test 용
        #         #if self.servo < servo_down:
        #         #   self.servo = self.servo + 20
        #         self.servo  = servo_down
        #         print("________get down________ ")
        
        
    def cruise(self):
        
        """
        diff = self.distance[1] - self.distance[0] # right - left
        
        if abs(diff) < threshold:
            self.substate = 2
            
        else:
            self.motor_cmd[0] = K_align * diff
            self.motor_cmd[1] = V_align
            """
        K = 0.01
        self.motor_cmd[0] -= K * self.w_z
        self.substate = 0

    
            
    def align(self):
        
        """
        diff = self.distance[1] - self.distance[0] # right - left
        
        if abs(diff) < threshold:
            self.substate = 2
            
        else:
            self.motor_cmd[0] = K_align * diff #
            self.motor_cmd[1] = V_align
            """
        if not (self.distance[0]<0 and self.distance[1]<0):
            if self.distance[0] - self.distance[1]> 1:
                self.motor_cmd[0] = w_align
                self.motor_cmd[1] = v_align
            elif self.distance[0] - self.distance[1]< -1:
                self.motor_cmd[0] = -w_align
                self.motor_cmd[1] = v_align
            else :
                self.substate = 0

            

    def climb(self):
        self.motor_cmd[0] = w_climb
        self.motor_cmd[1] = v_climb
        
        if self.euler[1] < threshold_climb: #finish climbing
            self.substate = 3
            if self.trial == 3:
                self.state = 0
            
    def turn(self):
        if self.yaw == None:
            self.yaw = self.euler[2]
        
        if abs(abs(self.euler[2]-self.yaw)- 3.1415926535/2) > threshold_turn:
            self.motor_cmd[0] = w_turn
            self.motor_cmd[1] = v_turn
            print(111111111111111111)
        else :
            self.motor_cmd[0] = 0
            self.motor_cmd[1] = 0
            self.substate = 0
            self.yaw = None
            
            

def sigmoid_mapping(x):
    return 2 / (1 + np.exp(-x)) - 1

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    # minimal_subscriber.actuate()
    #start spin
    rclpy.spin(minimal_subscriber)
    # msg.linear_acceleration_covariance[0]
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()

    print(minimal_subscriber.euler)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
