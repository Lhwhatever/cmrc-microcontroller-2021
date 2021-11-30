import time
import math
import board
import busio
import smbus2
import adafruit_bno055
import altimeter
import picamera
from CircularBuffer import CircularBuffer

RPI_BUS_NUM = 1
MULTIPLEXER_ADDR = 0x70

VID_OUTPUT = 'yinzer.h264'
I2C_CH = [1 << i for i in range(8)]

G_ACC = 9.81
TICKRATE = 10.0

ALTI_OUTPUT = 'yinzer_alti.csv'
IMU_OUTPUT = 'yinzer_imu.csv'

##############################################################
# GLOBAL VARIABLES
##############################################################

bus = smbus2.SMBus(RPI_BUS_NUM)
i2c = busio.I2C(board.SCL, board.SDA)

mux_addrs = {
        'imu': 7,
        'alti1': 6,
        'alti2': 5,
        'alti3': 4,
        }

camera = picamera.PiCamera()

f_alti = None
f_imu = None

alti = CircularBuffer(10.0 * TICKRATE)

##############################################################
# HELPER FUNCTIONS
##############################################################

def magnitude(v):
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

def get_barometer():
    pass

def get_acceleration():
    bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[mux_addrs['imu']])
    return sensors['imu'].acceleration

def noop():
    pass

##############################################################
# HELPER FUNCTIONS
##############################################################

# PRE-LAUNCH

def setup():
    bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[mux_addrs['imu']])
    sensors['imu'] = adafruit_bno055.BNO055_I2C(i2c)
    for x in ['alti1', 'alti2', 'alti3']:
        bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[mux_addrs[x]])
        sensors[x] = altimeter.Altimeter(i2c)

    camera.resolution = (1280, 720)
    camera.framerate = 60
    camera.start_recording(VID_OUTPUT)

    with open(ALTI_OUTPUT, 'w') as f:
        f.write('time,altitude,pressure,temp\n')

    with open(IMU_OUTPUT, 'w') as f:
        f.write('time,ax,ay,az')

    
    f_imu = open(IMU_OUTPUT, 'w')

def has_launched():
    acc = get_acceleration()
    return magnitude(acc) > 2 * G_ACC

# LIFT-OFF

def setup():
    f_alti = open(ALTI_OUTPUT, 'a')
    f_imu = open(IMU_OUTPUT, 'a')

def liftoff_loop():
    f_alti.write(f'{}')

def has_reached_apogee():
    pass
