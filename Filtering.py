import utime
import micropython
from machine import I2C, Pin
from mpu9250 import MPU9250

xAcc = 0
yAcc = 0
zAcc = 0
xGyro = 0
yGyro = 0
zGyro = 0
xMag = 0
yMag = 0
zMag = 0
Temp = 0