import utime
import micropython
from machine import I2C, Pin
from mpu9250 import MPU9250

x_pos = 0
y_pos = 0
z_pos = 0
x_vel = 0
y_vel = 0
z_vel = 0
x_acc = 0
y_acc = 0
z_acc = 0

def final_pos:
    x_pos = 0