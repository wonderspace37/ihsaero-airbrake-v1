import micropython
import os
from machine import Pin, SoftSPI, I2C
from sdcard import SDCard
from imu_libs.mpu9250 import MPU9250
from imu_libs.ak8963 import AK8963
from imu_libs.mpu6050 import MPU6050
import numpy as np

acc_Raw = np.array([0, 0, 0])
gyro_Raw = np.array([0, 0, 0])
mag_Raw = np.array([0, 0, 0, 0, 0])
temp_Raw = 0

acc_Filtered = np.array([0, 0, 0])
gyro_Filtered = np.array([0, 0, 0])
mag_Filtered = np.array([0, 0, 0])
temp_Filtered = 0

sclPin = 22
sdaPin = 21
class Store_Data:
    def Read_Setup():
        micropython.alloc_emergency_exception_buf(100)

        i2c = I2C(scl=Pin(sclPin), sda=Pin(sdaPin))

        dummy = MPU9250(i2c)  # this opens the bybass to access to the AK8963 and MPU 6050
        ak8963 = AK8963(i2c)
        mpu6050 = MPU6050(i2c)
        offset_mag, scale_mag = ak8963.calibrate(count=256, delay=200)
        offset_gyro, scale_gyro = mpu6050.calibrate(count=256, delay=200)

        sensor = MPU9250(i2c, ak8963=ak8963, mpu6500=mpu6500)
        print("MPU9250 id: " + hex(sensor.whoami))

        spisd = SoftSPI(miso=Pin(14), mosi=Pin(12), sck=Pin(13))
        sd = SDCard(spisd, Pin(10))

        print('root directory: {}'.format(os.listdir()))
        vfs = os.VfsFat(sd)
        os.mount(vfs, '/sd')
        print('root directory: {}'.format(os.listdir()))
        os.chdir('/sd')
        print('SD card contains: {}'.format(os.listdir()))

        f = open('/sd/Data.txt', 'w')
        f.write('Bitch we start')
        f.close()

    def readData():
        acc_Raw[0], acc_Raw[1], acc_Raw[2] = sensor.acceleration
        gyro_Raw[0], gyro_Raw[1], gyro_Raw[2] = sensor.gyroscope
        mag_Raw[0], mag_Raw[1], mag_Raw[2] = sensor.magnetic
        temp_raw = sensor.temperature

        print(acc_Raw[0], acc_Raw[1], acc_Raw[2])
        print(gyro_Raw[0], gyro_Raw[1], gyro_Raw[2])
        print(mag_Raw[0], mag_Raw[1], mag_Raw[2])
        print(temp_raw)

    def Store_Data_Raw():
        readData()
        f = open('/sd/Data.txt', 'a')
        f.write('acc data: ', acc_Raw, ' gyro data: ', gyro_Raw, ' mag data: ', mag_Raw)
        f.close()

    def Store_Data_Filtered():
        f = open('/sd/Data.txt', 'a')
        f.write('acc data: ', acc_Filtered, ' gyro data: ', gyro_, ' mag data: ', mag_Raw)
        f.close()

    def push_accx_Raw():
        return acc_Raw[0]
    def push_accy_Raw():
        return acc_Raw[1]
    def push_accz_Raw():
        return acc_Raw[2]

    def push_gyrox_Raw():
        return gyro_Raw[0]
    def push_gyroy_Raw():
        return gyro_Raw[1]
    def push_gyroz_Raw():
        return gyro_Raw[2]

    def push_magx_Raw():
        return mag_Raw[0]
    def push_magy_Raw():
        return mag_Raw[1]
    def push_magz_Raw():
        return mag_Raw[2]

    def push_accx_Filtered():
        return acc_Filtered[0]
    def push_accy_Filtered():
        return acc_Filtered[1]
    def push_accz_Filtered():
        return acc_Filtered[2]

    def push_gyrox_Filtered():
        return gyro_Filtered[0]
    def push_gyroy_Filtered():
        return gyro_Filtered[1]
    def push_gyroz_Filtered():
        return gyro_Filtered[2]

    def push_magx_Filtered():
        return mag_Filtered[0]
    def push_magy_Filtered():
        return mag_Filtered[1]
    def push_magz_Filtered():
        return mag_Filtered[2]

    def push_temp_Raw():
        return temp_Raw

    def push_temp_Filtered():
        return temp_Filtered