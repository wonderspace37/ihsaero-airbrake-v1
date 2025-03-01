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

def readData():
    

def Store_Raw_Data():
    x = Pin, SoftSPI


def Store_Data_Filtered():
    x = x
