import rclpy
import smbus 
import math
import time
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from imu_pub.transformations import euler_to_quaternion

import RPi.GPIO as GPIO 


#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

#sonar Registers and their Address
trig_1 = 11
echo_1 = 8
trig_2 = 27
echo_2 = 17
GPIO.cleanup()   
GPIO.setmode(GPIO.BCM)                     # GPIO 이름은 BCM 명칭 사용
GPIO.setup(trig_1, GPIO.OUT)                   # Trig=17 초음파 신호 전송핀 번호 지정 및 출력지정
GPIO.setup(echo_1, GPIO.IN)                    # Echo=18 초음파 수신하는 수신 핀 번호 지정 및 입력지정
GPIO.setup(trig_2, GPIO.OUT)                   # Trig=17 초음파 신호 전송핀 번호 지정 및 출력지정
GPIO.setup(echo_2, GPIO.IN)                    # Echo=18 초음파 수신하는 수신 핀 번호 지정 및 입력지정


bus = smbus.SMBus(0) 
def MPU_Init(Device_Address = 0x68  ):
	
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7) #write to sample rate register

    #Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

    #Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)

    #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

    #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, ACCEL_CONFIG, 0)

    #Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)
        

def read_raw_data(addr, Device_Address = 0x68  ):
    #Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)
    #concatenate higher and lower value
    value = ((high << 8) | low)

    #to get signed value from mpu6050
    if(value > 32768):
            value = value - 65536
    return value


def sonar(trig,echo):
   start = time.time()
   stop  = time.time()
   while True :
            GPIO.output(trig, False)           
            time.sleep(0.01)
            GPIO.output(trig, True)          # 10us 펄스를 내보낸다.   
            time.sleep(0.0001)            # Python에서 이 펄스는 실제 100us 근처가 될 것이다
            GPIO.output(trig, False)         # 하지만 HC-SR04 센서는 이 오차를 받아 
            sensor_time = time.time()
            while GPIO.input(echo) == False:     # 18번 핀이 OFF 되는 시점을 시작 시간으로 잡는다
               start = time.time()
               if start - sensor_time > 0.001:
                    break
               #print(1)
            while GPIO.input(echo) == True:     # 18번 핀이 다시 ON 되는 시점을 반사파 수신시간으로 잡는다
               stop  = time.time()
               if  stop - sensor_time > 0.005:
                    stop = sensor_time
                    break
               #print(2)
            time_interval = stop - start      # 초음파가 수신되는 시간으로 거리를 계산한다
            distance = time_interval * 17000
            distance = round(distance, 2)

            if distance < 300  :
               return distance 


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Publishing IMU data")
        self.euler = (0,0,0)
        time_ =  Node('time_example_node').get_clock().now().to_msg()
        self.time_pre =time_.sec + time_.nanosec*0.1**9   
        self.i = 0
        
        self.acc_offset = (0,0,0)
        self.gyro_offset = (0,0,0)
        self.calibrate(100)
        print(self.acc_offset[0])
        print(self.acc_offset[1])
        print(self.acc_offset[2])
    

    def timer_callback(self):
        #Read Accelerometer raw value
        acc_x = read_raw_data(ACCEL_XOUT_H) - self.acc_offset[0]
        acc_y = read_raw_data(ACCEL_YOUT_H) - self.acc_offset[1]
        acc_z = read_raw_data(ACCEL_ZOUT_H) #- self.acc_offset[2]

        #Read Gyroscope raw value
        gyro_x = read_raw_data(GYRO_XOUT_H) - self.gyro_offset[0]
        gyro_y = read_raw_data(GYRO_YOUT_H) - self.gyro_offset[1]
        gyro_z = read_raw_data(GYRO_ZOUT_H) - self.gyro_offset[2]

        #Full scale range +/- 2000 degree/C as per sensitivity scale factor
        Ax = acc_x/16384.0
        Ay = acc_y/16384.0
        Az = -1.0 #acc_z/16384.0 - 1

        Gx = gyro_x/131.0*math.pi/180
        Gy = gyro_y/131.0*math.pi/180
        Gz = gyro_z/131.0*math.pi/180
        #Gz = gyro_z/1048.6

        imu_data = Imu()
        
       
        gyro = Vector3()
        gyro.x, gyro.y, gyro.z = Gx, Gy, Gz
        
        accel = Vector3()
        accel.x, accel.y, accel.z = Ax, Ay, Az
        
        imu_data.angular_velocity               =gyro
        #imu_data.angular_velocity_covariance[0] = sonar(trig_1 , echo_1) # LF sonar distance
        #imu_data.angular_velocity_covariance[4] = sonar(trig_2 , echo_2) # RF sonar distance
        imu_data.angular_velocity_covariance[8] =0.001
       
        imu_data.linear_acceleration              =accel
        imu_data.linear_acceleration_covariance[0] =self.euler[0]  # roll
        imu_data.linear_acceleration_covariance[4] =self.euler[1]  #pitch
        imu_data.linear_acceleration_covariance[8] =self.euler[2]  #yaw
        
        Gyro =(Gx, Gy, Gz)
        Acc =( Ax, Ay, Az)
        imu_data.header.stamp  = Node('time_example_node').get_clock().now().to_msg()  
        #print(imu_data.header.stamp)
        #time_now = imu_data.header.stamp.sec + imu_data.header.stamp.nanosec*0.1**9
        time_now = time.time()
        dt = time_now - self.time_pre
        self.time_pre = time_now
        
        
        self.euler = comp_filter(alpha=0.98, acc=Acc, gyro=Gyro, euler=self.euler,dt =dt)
        x,y,z,w = euler_to_quaternion(self.euler) 
        imu_data.orientation.x, \
        imu_data.orientation.y, \
        imu_data.orientation.z, \
        imu_data.orientation.w = x, y, z, w
          
        imu_data.header.frame_id  = 'imu'  
        print(imu_data.angular_velocity_covariance[0])
        print(imu_data.angular_velocity_covariance[4])
        print(self.euler)
        print("Accz: " + str(Az))
        print(1111)
        self.publisher_.publish(imu_data)
        #
        self.i += 1
    
    def calibrate(self, num_samples):
        x_accel = 0
        y_accel = 0
        z_accel = 0
        x_gyro = 0
        y_gyro = 0
        z_gyro = 0
        for i in range(num_samples):
            acc_x = read_raw_data(ACCEL_XOUT_H)
            acc_y = read_raw_data(ACCEL_YOUT_H)
            acc_z = read_raw_data(ACCEL_ZOUT_H)
            gyro_x = read_raw_data(GYRO_XOUT_H)
            gyro_y = read_raw_data(GYRO_YOUT_H)
            gyro_z = read_raw_data(GYRO_ZOUT_H)
            
            x_accel += acc_x
            y_accel += acc_y
            z_accel += acc_z
            x_gyro += gyro_x
            y_gyro += gyro_y
            z_gyro += gyro_z
        
        x_accel /= num_samples
        y_accel /= num_samples
        z_accel /= num_samples
        x_gyro /= num_samples
        y_gyro /= num_samples
        z_gyro /= num_samples
        
        self.acc_offset = (x_accel, y_accel, z_accel)
        self.gyro_offset = (x_gyro, y_gyro, z_gyro)
            
        
        
        
def comp_filter(alpha, acc, gyro, euler,dt):

    ax = acc[0]
    ay = acc[1]
    az = acc[2]

    gx = gyro[0]
    gy = gyro[1]
    gz = gyro[2]

    roll, pitch, yaw = euler

    pitch_acc = math.atan(-ax/math.sqrt(ay**2 + az**2))
    roll_acc = math.atan(ay / math.sqrt(ax**2 + az**2))
    #roll_acc = math.atan(ay / math.sqrt(az**2))

    roll_gyro = roll + (gx * dt)
    pitch_gyro = pitch + (gy * dt)
    yaw_gyro = yaw + (gz * dt)

    roll = alpha * roll_gyro + (1-alpha) * roll_acc
    pitch = alpha * pitch_gyro + (1-alpha) * pitch_acc
    yaw = yaw_gyro
    
    return roll, pitch, yaw
    
    
    


def main(args=None):
    bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
    Device_Address = 0x68   # MPU6050 device address
    MPU_Init()
    print (" Reading Data of Gyroscope and Accelerometer")

    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
