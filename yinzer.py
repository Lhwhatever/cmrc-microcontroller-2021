#!/usr/bin/python3

import time
from datetime import datetime
import math
from CircularBuffer import CircularBuffer
import board
import busio
import smbus2
import adafruit_bno055
import altimeter
import picamera
from collections import deque

RPI_BUS_NUM = 1
MULTIPLEXER_ADDR = 0x70

VID_OUTPUT = 'yinzer.h264'
I2C_CH = [1 << i for i in range(8)]

G_ACC = 9.81
TICKRATE = 10.0

ALTI_OUTPUT = 'yinzer_alti.csv'
IMU_OUTPUT = 'yinzer_imu.csv'

HALF_SEC = int(0.5 * TICKRATE)
TWO_SEC = int(2 * TICKRATE)

##############################################################
# HELPER FUNCTIONS
##############################################################

def magnitude(v):
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

def write_row(csv_f, data):
    # csv_f is a write-capable file handler to a CSV file
    # data is an iterable
    csv_f.write(','.join(map(str, data)) + '\n')

def noop():
    pass


##############################################################
# CLASS DECLARATIONS
##############################################################

# In a class so that we can write to global variables from inside functions
class Microcontroller:
    bus = smbus2.SMBus(RPI_BUS_NUM)
    i2c = busio.I2C(board.SCL, board.SDA)

    mux_addrs = {
        'imu': 7,
        'alti1': 6,
        'alti2': 5,
        'alti3': 4,
        }

    camera = picamera.PiCamera()

    sensors = {}

    data: CircularBuffer

    def __init__(self):
        self.f_alti = open(ALTI_OUTPUT, 'w')
        self.f_imu = open(IMU_OUTPUT, 'w')
        self.data = CircularBuffer(int(10 * TICKRATE))

    def __del__(self):
        self.f_imu.close()
        self.f_alti.close()

    def get_acceleration(self):
        self.bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[self.mux_addrs['imu']])
        return self.sensors['imu'].linear_acceleration

    def get_imu_all(self):
        self.bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[self.mux_addrs['imu']])
        return {
            'acc': self.sensors['imu'].linear_acceleration,
            'gyr': self.sensors['imu'].gyroscope,
            'qua': self.sensors['imu'].quaternion
            }

    def get_barometer(self):
        self.bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[self.mux_addrs['alti1']])
        p1 = self.sensors['alti1'].pressure
        a1 = self.sensors['alti1'].altitude
        self.bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[self.mux_addrs['alti2']])
        p2 = self.sensors['alti2'].pressure
        a2 = self.sensors['alti2'].altitude
        self.bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[self.mux_addrs['alti3']])
        p3 = self.sensors['alti3'].pressure
        a3 = self.sensors['alti3'].altitude
        return {
            'pressure': (p1 + p2 + p3) / 3,
            'altitude': (a1 + a2 + a3) / 3
            }


##############################################################
# FUNCTIONS FOR EACH STAGE
##############################################################

# PRE-LAUNCH
LIFTOFF_THRESHOLD = 2 * G_ACC

def prelaunch_setup(m: Microcontroller):
    m.bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[m.mux_addrs['imu']])
    m.sensors['imu'] = adafruit_bno055.BNO055_I2C(m.i2c)
    for x in ['alti1', 'alti2', 'alti3']:
        m.bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[m.mux_addrs[x]])
        m.sensors[x] = altimeter.Altimeter(m.i2c)

    m.camera.resolution = (1280, 720)
    m.camera.framerate = 60
    m.camera.start_recording(VID_OUTPUT)

    m.f_alti.write('time,altitude,pressure\n')
    m.f_imu.write('time,ax,ay,az,gx,gy,gz,qw,qx,qy,qz\n')

    with open(ALTI_OUTPUT, 'w') as f:
        f.write

    with open(IMU_OUTPUT, 'w') as f:
        f.write

    m.f_alti = open(ALTI_OUTPUT, 'w')
    m.f_imu = open(IMU_OUTPUT, 'w')

def prelaunch_check(m: Microcontroller):
    # check if magnitude of linear acceleration has been consistently greater 
    # than threshold for the preceding 0.5 sec
    m.data.append(m.get_acceleration())
    return all(x >= LIFTOFF_THRESHOLD for x in m.data[-HALF_SEC:])

# LIFT-OFF

def liftoff_setup(m: Microcontroller):
    m.data = CircularBuffer(int(10 * TICKRATE))

def liftoff_loop(m: Microcontroller, now: datetime):
    timestamp = now.astimezone().isoformat(sep=' ', timespec='milliseconds')
    imu = m.get_imu_all()
    bar = m.get_barometer()
    m.data.append((imu, bar))
    write_row(m.f_alti, [timestamp, bar['altitude'], bar['pressure']])
    write_row(m.f_imu, [
            timestamp,
            imu['acc'].x, imu['acc'].y, imu['acc'].z,
            imu['gyr'].x, imu['gyr'].y, imu['gyr'].z,
            imu['qua'].w, imu['qua'].x, imu['qua'].y, imu['qua'].z,
            ])


def liftoff_check(m: Microcontroller):
    if len(m.data) >= TWO_SEC:
        _, bar_now = m.data[-1]
        _, bar_old = m.data[-TWO_SEC]
        return bar_old['altitude'] > bar_now['altitude']
    return False


##############################################################
# RUNTIME
##############################################################

# Each stage is represented by a 4-tuple
# 1. Name of stage (for print/debug purposes)
# 2. Setup function: To be run ONCE when entering this stage
# 3. Loop function: To be run every tick during this stage
# 4. Check function: To check every tick whether we have entered the next 
#    stage (returns True in that case)

stages = [
        ("Pre-Launch", prelaunch_setup, noop, prelaunch_check),
        ("Liftoff", liftoff_setup, liftoff_loop, liftoff_check)
        ]

def main():
    print("Setting up microcontroller")
    m = Microcontroller()
    SECONDS_PER_TICK = 1 / TICKRATE

    for name, setup, loop, check  in stages:
        timestamp = datetime.now().astimezone()\
                .isoformat(sep=' ', timespec='milliseconds')
        print(f"{timestamp}: Entering stage {name}")
        setup(m)
        while not check(m):
            loop(m, datetime.now())
            m.camera.wait_recording(SECONDS_PER_TICK)


if __name__ == '__main__':
    main()
