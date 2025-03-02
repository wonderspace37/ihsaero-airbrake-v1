import os
from machine import Pin, SoftSPI
from sdcard import SDCard
from imu_libs.mpu9250 import MPU9250
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

micropython.alloc_emergency_exception_buf(100)

i2c = I2C(scl=Pin(sclPin), sda=Pin(sdaPin))

dummy = MPU9250(i2c)  # this opens the bybass to access to the AK8963 and MPU 6050
ak8963 = AK8963(i2c)
mpu6050 = MPU6050(i2c)
offset_mag, scale_mag = ak8963.calibrate(count=256, delay=200)
offset_gyro, scale_gyro = mpu6050.calibrate(count=256, delay=200)

sensor = MPU9250(i2c, ak8963=ak8963, mpu6500=mpu6500)

def readData():
    acc_Raw[0], acc_Raw[1], acc_Raw[2] = sensor.acceleration
    gyro_Raw[0], gyro_Raw[1], gyro_Raw[2] = sensor.gyroscope
    mag_Raw[0], mag_Raw[1], mag_Raw[2] = sensor.magnetic
    temp_raw = sensor.temperature

    print(acc_Raw[0], acc_Raw[1], acc_Raw[2]),
    print(gyro_Raw[0], gyro_Raw[1], gyro_Raw[2]),
    print(mag_Raw[0], mag_Raw[1], mag_Raw[2]),
    print(temp_raw)

def Store_Raw_Data():
    x = x


def Store_Data_Filtered():
    x = x
