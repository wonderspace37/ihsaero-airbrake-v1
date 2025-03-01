# from machine import I2C, Pin, Timer
from imu_libs.mpu9250 import MPU9250


Debug = False
rocketLaunching = False
ApogeeReached = False
RocketArmed = False

accx_raw = 0
accy_raw = 0
accz_raw = 0
gyrox_raw = 0
gyroy_raw = 0
gyroz_raw = 0
magnetx_raw = 0
magnety_raw = 0
magnetz_raw = 0
temp_raw = 0

cycleDelay = 500
rocketMode = 0

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

print("MPU9250 id: " + hex(sensor.whoami))


def read_sensor():
    accx_raw, accy_raw, accz_raw = sensor.acceleration
    gyrox_raw, gyroy_raw, gyroz_raw = sensor.gyroscope  # check
    magnetx_raw, magnety_raw, magnetz_raw = sensor.magnetic
    temp_raw = sensor.temperature

    print(accx_raw, accy_raw, accz_raw),
    print(gyrox_raw, gyroy_raw, gyroz_raw),
    print(magnetx_raw, magnety_raw, magnetz_raw),
    print(temp_raw)


def value_filter():
    x = x


def mode_Set():
    # Set/Figure out Mode
    x = x


def mode_Switch():

    # Always happens

    if rocketLaunching == True:
        # Rocket inactive
        x = x
    elif ApogeeReached == True:
        x = x
        # Rocket armed
    elif RocketArmed == True:
        x = x
        # Rocket launched


def run_Program(timer):
    read_sensor()
    value_filter()
    mode_Set()
    mode_Switch()


timer_0 = Timer(0)
# period controlls time in ms, callback runs program
timer_0.init(period=1000, mode=Timer.PERIODIC, callback=run_Program)
