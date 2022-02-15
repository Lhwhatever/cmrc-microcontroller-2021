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

TICKRATE = 20.0

ALTI_OUTPUT = 'yinzer_alti.csv'
IMU_OUTPUT = 'yinzer_imu.csv'

# Units
G_ACCELERATION = 9.81
SECOND = TICKRATE
METER = 1.0
FEET = 0.3048

##############################################################
# PARAMETERS
##############################################################

CIRCULAR_BUFFER_SIZE = int(5 * SECOND)

# for IIR

IIR_WT = 0.1

# lift-off detection parameters
# program will begin lift-off code once acceleration exceeds the following
# threshold, and is sustained for the following duration

LIFTOFF_DETECTION_ACC = 2 * G_ACCELERATION
LIFTOFF_DETECTION_TIME = int(0.5 * SECOND)
COUNT_PCT = 0.95


# apogee detection parameters
# program will begin descent code if the altitude now is lower than the
# altitude _____ seconds ago, where ______ is specified below

APOGEE_DETECTION_TIME = int(2 * SECOND)


# main parachute deployment parameters
# program will deploy main parachute once the altitude falls below the 
# following threshold

MAIN_CHUTE_THRESHOLD = 500 * METER


# landing detection parameters
# program will end once the altitude _____ seconds ago is the same as the
# altitude now, where ____ is specified below

LANDING_DETECTION_TIME = int(4 * SECOND)


##############################################################
# HELPER FUNCTIONS
##############################################################

def magnitude(v):
    vx, vy, vz = v
    return math.sqrt(vx * vx + vy * vy + vz * vz)

def write_row(csv_f, data):
    # csv_f is a write-capable file handler to a CSV file
    # data is an iterable
    csv_f.write(','.join(map(str, data)) + '\n')

def noop(_, __):
    pass

def update_feature(new, old):
    # calculates the new feature to store
    return old * (1. - IIR_WT) + new * IIR_WT

def update_reading(new, old):
    # same as above, but vectorized (new and old are tuples of the same length
    # representing vectors)
    return tuple(map(lambda x: update_feature(x[0], x[1]), zip(new, old)))


##############################################################
# CLASS DECLARATIONS
##############################################################

# In a class to avoid nonsense with global variables
class Microcontroller:
    bus = smbus2.SMBus(RPI_BUS_NUM)
    i2c = busio.I2C(board.SCL, board.SDA)

    mux_addrs = {
        'imu': 2,
        'alti1': 1,
        'alti2': 5,
        'alti3': 6,
        }

    camera = picamera.PiCamera()

    sensors = {}

    data: CircularBuffer

    def __init__(self):
        self.f_alti = open(ALTI_OUTPUT, 'w')
        self.f_imu = open(IMU_OUTPUT, 'w')
        self.data = CircularBuffer(CIRCULAR_BUFFER_SIZE)
        self.rtdata = CircularBuffer(CIRCULAR_BUFFER_SIZE)

    def update(self, now: datetime):
        timestamp = now.astimezone().isoformat(sep=' ', timespec='milliseconds')

        imu = self.get_imu_all()
        bar = self.get_barometer()

        self.rtdata.append({ 'imu': imu, 'bar': bar })

        # write to file
        write_row(self.f_alti, [timestamp, bar['alt'], bar['bar']])
        write_row(self.f_imu, [timestamp, *imu['acc'], *imu['gyr'], *imu['qua']])

        # store moving average
        if len(self.data) > 0:
            old = self.data[-1]
            for key in ['acc', 'gyr', 'qua']:
                imu[key] = update_reading(imu[key], old['imu'][key])
            bar['alt'] = update_feature(bar['alt'], old['bar']['alt'])
            bar['bar'] = update_feature(bar['bar'], old['bar']['bar'])

        self.data.append({ 'imu': imu, 'bar': bar })

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
            'gyr': self.sensors['imu'].gyro,
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
            'bar': (p1 + p2 + p3) / 3,
            'alt': (a1 + a2 + a3) / 3
            }


##############################################################
# FUNCTIONS FOR EACH STAGE
##############################################################

# PRE-LAUNCH

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
    m.f_imu.write('time,ax,ay,az,gx,gy,gz,qx,qy,qz,qw\n')

def prelaunch_loop(m: Microcontroller, now: datetime):
    m.update(now)

def prelaunch_check(m: Microcontroller):
    if len(m.data) <= LIFTOFF_DETECTION_TIME: return False

    hits = sum(map(lambda data: 1 if magnitude(data['imu']['acc']) > LIFTOFF_DETECTION_ACC else 0, m.data[-LIFTOFF_DETECTION_TIME:]))

    return (hits / LIFTOFF_DETECTION_TIME) > COUNT_PCT


# LIFT-OFF

def liftoff_setup(m: Microcontroller):
    pass

def liftoff_loop(m: Microcontroller, now: datetime):
    # Log IMU and altimeter data
    m.update(now)

def liftoff_check(m: Microcontroller):
    # Check if the altitude now is lower than the altitude some time ago
    # (some time is specified in the parameters)
    if len(m.data) >= APOGEE_DETECTION_TIME:
        bar_now = m.data[-1]['bar']
        bar_old = m.data[-APOGEE_DETECTION_TIME]['bar']
        if bar_old['alt'] > bar_now['alt']:
            print(f"Apogee (1 second ago): {bar_old['alt']}")
            return True
    return False


# DESCENT PT. 1

def descent_1_setup(m: Microcontroller):
    # Deploy drogue chutes
    # TODO!!!
    print("Warning: drogue chute deployment not implemented yet")

def descent_1_loop(m: Microcontroller, now: datetime):
    # Same as liftoff
    return liftoff_loop(m, now)

def descent_1_check(m: Microcontroller):
    # Check that the altitude is lower than a certain threshold
    return m.data[-1]['bar']['alt'] < MAIN_CHUTE_THRESHOLD


# DESCENT PT. 2

def descent_2_setup(m: Microcontroller):
    # Deploy main chutes
    # TO-DO!!!
    print("Warning: main chute deployment not implemented yet")

def descent_2_loop(m: Microcontroller, now: datetime):
    # Same as liftoff
    return liftoff_loop(m, now)

def descent_2_check(m: Microcontroller):
    # Check that the altitude has not changed for a certain time
    alt_now = m.data[-1]['bar']['alt']
    alt_old = m.data[-LANDING_DETECTION_TIME]['bar']['alt']

    # Due to floating-point arithmetic we use 1mm as our threshold
    return abs(alt_now - alt_old) < 0.001


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
        ("Pre-Launch", prelaunch_setup, prelaunch_loop, prelaunch_check),
        ("Liftoff", liftoff_setup, liftoff_loop, liftoff_check),
        ("Descent 1", descent_1_setup, descent_1_loop, descent_1_check),
        ("Descent 2", descent_2_setup, descent_2_loop, descent_2_check)
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
