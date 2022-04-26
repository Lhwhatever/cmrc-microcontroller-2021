import time
from datetime import datetime, timedelta
import RPi.GPIO as GPIO
import math
import busio
import smbus2
import adafruit_bno055
import altimeter
import stepper
import os
import multiprocessing as mp

TICKRATE = 20.0

# Units
G_ACCELERATION = 9.81
SECOND = TICKRATE
METER = 1.0
FEET = 0.3048


DROGUE_PIN = 23
MAIN_PIN = 24
BUZZ_PIN = 26

IMU_ADDR = 0x28
BAR_ADDR = 0x60

IIR_WT = 0.2

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

MAIN_CHUTE_THRESHOLD = 500 * FEET


# landing detection parameters
# program will end once the altitude _____ seconds ago is the same as the
# altitude now, where ____ is specified below

LANDING_DETECTION_TIME = int(4 * SECOND)


# timers for backup!!!
# timedelta required to begin each stage IF sensors break
# start counting from liftoff

TIME_DESCENT_1 = timedelta(seconds=20)
TIME_DESCENT_2 = timedelta(seconds=70)

# seconds to disable output to igniter pin

CLEAN_IGNITERS_TIME = timedelta(seconds=5)

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

##############################################################
# CLASS DECLARATIONS
##############################################################

# In a class to avoid nonsense with global variables
class Microcontroller:
    GPIO.setmode(GPIO.BCM)
    i2c = busio.I2C(3, 2) # SLA, SDC

    sensors = {}
    
    def __init__(self):
        name_counter = 0
        self.alti_output = '/home/pi/Desktop/andrews-gambit-flight-data/'
        self.imu_output = '/home/pi/Desktop/andrews-gambit-flight-data/'
        while True:
            alti_output_temp = self.alti_output + str(name_counter) + '-yinzer_alti.csv'
            imu_output_temp = self.imu_output + str(name_counter) + '-yinzer_imu.csv'
            try:
                open(alti_output_temp, 'x')
                open(imu_output_temp, 'x')
                
                self.alti_output = alti_output_temp
                self.imu_output = imu_output_temp
                break
            except FileExistsError:
                name_counter += 1
        
        self.f_alti = open(self.alti_output, 'w')
        self.f_imu = open(self.imu_output, 'w')
        self.liftoff = None
        self.alt_offset = 0
        
        self.motor = None
        
        self.alt_iir = 0
        
        self.kP = 10
        self.kI = 0
        
        self.prelauch_check_counter = 0
        self.liftoff_check_counter = 0
        self.last_alt = None
        self.descent_1_check_counter = 0
        self.descent_2_check_counter = 0
        
        GPIO.setup(DROGUE_PIN, GPIO.OUT)
        GPIO.setup(MAIN_PIN, GPIO.OUT)
        GPIO.setup(BUZZ_PIN, GPIO.OUT)
        
        self.drogue_fired = None
        self.main_fired = None
        self.save_counter = 0
        
    def update(self, now: datetime):
        timestamp = now.astimezone().isoformat(sep=' ', timespec='milliseconds')

        imu = self.get_imu_all()
        bar = self.get_barometer()

        # write to file
        write_row(self.f_alti, [timestamp, bar['alt']])
        write_row(self.f_imu, [timestamp, *imu['acc'], *imu['gyr'], *imu['qua']])
        
    def __del__(self):
        self.f_imu.close()
        self.f_alti.close()
        
    def get_acceleration(self):
        ret_val = self.sensors['imu'].linear_acceleration
        for value in ret_val:
            if value is None:
                return (0, 0, 0)
        return ret_val

    def get_imu_all(self):
        ret_dict = {
            'acc': self.sensors['imu'].linear_acceleration,
            'gyr': self.sensors['imu'].gyro,
            'qua': self.sensors['imu'].quaternion
            }
        if ret_dict['acc'] is None: ret_dict['acc'] = 0
        if ret_dict['acc'] is None: ret_dict['gyr'] = 0
        
        return ret_dict
    
    def get_barometer(self):
        try:
            #p = self.sensors['alti1'].pressure
            alt = self.sensors['alt'].altitude
        except:
            #p = -50000
            alt = -50000
        if alt is None:
            return self.alt_iir
        
        #print(self.sensors['imu'].euler)
        alt = alt - self.alt_offset
        alt = IIR_WT*alt + (1 - IIR_WT)*self.alt_iir
        
        self.alt_iir = alt
        #print(alt)
        return {
            'alt': alt
            }
    
        # disable if pins have been fired for some time
    def clean(self, time):
        if self.drogue_fired is not None:
            if time - self.drogue_fired > CLEAN_IGNITERS_TIME:
                GPIO.output(DROGUE_PIN, GPIO.LOW)
                self.drogue_fired = None

        if self.main_fired is not None:
            if time - self.main_fired > CLEAN_IGNITERS_TIME:
                GPIO.output(MAIN_PIN, GPIO.LOW)
                self.main_fired = None
                
def stepperInit(m: Microcontroller):
    # runs init code for steppers, should be run in the descent_2_setup
    m.motor = stepper.Stepper(11, 27, 22, 10, 9)
    m.motor.set_resolution("1/2")
    m.motor.start_motor()


def stepperPID(m: Microcontroller, turn_rate):
    # specify ideal turn rate in ??? (0 results in no motion)

    # and compare with setpoint to get error
    omega = m.get_imu_all()['gyr'][0]
    error = turn_rate - omega

    # pi time
    p = error
    i = 0
    
    controlOut = m.kP*p + m.kI*i

    # motor time bois!!
    # each step should take 0.05 sec
    if controlOut < 0:
        stepDir = 0
    else:
        stepDir = 1
    
    motorSteps = int(abs(controlOut))
    stepFreq = int(motorSteps/0.05) + 1

    m.motor.move_motor(motorSteps, stepDir, stepFreq)

def prelaunch_setup(m: Microcontroller):
    GPIO.output(BUZZ_PIN, GPIO.HIGH)
    time.sleep(.5)
    GPIO.output(BUZZ_PIN, GPIO.LOW)
    time.sleep(.2)
    GPIO.output(BUZZ_PIN, GPIO.HIGH)
    time.sleep(.2)
    GPIO.output(BUZZ_PIN, GPIO.LOW)
    time.sleep(.2)
    GPIO.output(BUZZ_PIN, GPIO.HIGH)
    time.sleep(.2)
    GPIO.output(BUZZ_PIN, GPIO.LOW)
    
    m.sensors['imu'] = adafruit_bno055.BNO055_I2C(m.i2c)
    m.sensors['alt'] = altimeter.Altimeter(m.i2c)    
    
    m.f_alti.write('time,altitude,pressure\n')
    m.f_imu.write('time,ax,ay,az,gx,gy,gz,qx,qy,qz,qw\n')
    
    for i in range(10):
        m.sensors['alt'].altitude
        
    avg_height = 0
    for i in range(20):
        avg_height += m.sensors['alt'].altitude
        
    m.alt_offset = avg_height/20
    #print(m.alt_offset)
    print("pre-launch setup done")
    
    
def prelaunch_loop(m: Microcontroller, now: datetime):
    m.update(now)
    
def prelaunch_check(m: Microcontroller):
    accel = magnitude(m.get_acceleration())
    if accel >= LIFTOFF_DETECTION_ACC:
        m.prelauch_check_counter += 2
    else:
        m.prelauch_check_counter -= 1
    return m.prelauch_check_counter > 20

def liftoff_setup(m: Microcontroller):
    m.liftoff = datetime.now()

def liftoff_loop(m: Microcontroller, now: datetime):
    # Log IMU and altimeter data
    m.update(now)
    
def liftoff_check(m: Microcontroller):
    if m.last_alt == None:
        m.last_alt = m.get_barometer()
        return False
    
    alt = m.get_barometer()
    if alt['alt'] < m.last_alt['alt']:
        m.liftoff_check_counter += 2
        m.last_alt['alt'] = alt['alt']
    else:
        m.liftoff_check_counter -= 1
        m.last_alt['alt'] = alt['alt']
    return m.liftoff_check_counter > 40

def descent_1_setup(m: Microcontroller):
    stepperInit(m)
    # Deploy drogue chutes
    # TODO!!!
    GPIO.output(DROGUE_PIN, GPIO.HIGH)
    print("Warning: drogue chute deployment not tested yet")

def descent_1_loop(m: Microcontroller, now: datetime):
    result = liftoff_loop(m, now)
    stepperPID(m, 1)
    m.clean(now)
    return result

def descent_1_check(m: Microcontroller):
    if m.get_barometer()['alt'] < MAIN_CHUTE_THRESHOLD:
        m.descent_1_check_counter += 2
    else:
        m.descent_1_check_counter -= 1
    return m.descent_1_check_counter > 20

def descent_2_setup(m: Microcontroller):
    # Deploy main chutes
    # TODO!!!
    m.motor.stop_motor()
    GPIO.output(MAIN_PIN, GPIO.HIGH)
    print("Warning: main chute deployment not implemented yet")

def descent_2_loop(m: Microcontroller, now: datetime):
    result = liftoff_loop(m, now)
    m.clean(now)
    return result
    
def descent_2_check(m: Microcontroller):
    if m.last_alt == None:
        m.last_alt = m.get_barometer()
        return False
    
    alt = m.get_barometer()
    if (alt['alt'] - m.last_alt['alt']) < 0.001:
        m.descent_2_check_counter += 2
        m.last_alt['alt'] = alt['alt']
    else:
        m.descent_2_check_counter -= 1
        m.last_alt['alt'] = alt['alt']
    return m.descent_2_check_counter > 40





# Each stage is represented by a 5-tuple
# 1. Name of stage (for print/debug purposes)
# 2. Setup function: To be run ONCE when entering this stage
# 3. Loop function: To be run every tick during this stage
# 4. Check function: To check every tick whether we have entered the next 
# 5. Minimum timedelta from liftoff needed
#    stage (returns True in that case)

stages = [
        ("Pre-Launch", prelaunch_setup, prelaunch_loop, prelaunch_check, None),
        ("Liftoff", liftoff_setup, liftoff_loop, liftoff_check, TIME_DESCENT_1),
        ("Descent 1", descent_1_setup, descent_1_loop, descent_1_check, TIME_DESCENT_2),
        ("Descent 2", descent_2_setup, descent_2_loop, descent_2_check, None)
        ]


def compare_time(m, delta):
    if m.liftoff is None or delta is None:
        return False
    if datetime.now() - m.liftoff >= delta:
        print("Timer elapsed, automatically moving to next stage!")
        return True
    return False


def main():
    print("Setting up microcontroller")
    m = Microcontroller()
    SECONDS_PER_TICK = 1 / TICKRATE

    for name, setup, loop, check, delta in stages:
        timestamp = datetime.now().astimezone()\
                .isoformat(sep=' ', timespec='milliseconds')
        print(f"{timestamp}: Entering stage {name}")
        setup(m)
        while not (check(m) or compare_time(m, delta)):
            t0 = time.time()
            loop(m, datetime.now())
            time.sleep(SECONDS_PER_TICK)
            
            if m.save_counter%100 == 0:
                m.f_imu.close()
                m.f_alti.close()
                m.f_imu = open(m.imu_output, 'a')
                m.f_alti = open(m.alti_output, 'a')
            m.save_counter += 1
            # assert(False)
            #print(time.time()-t0)
            # m.camera.wait_recording(SECONDS_PER_TICK)
    
    #os.system("sudo shutdown -h now")


if __name__ == '__main__':
    try:
        # lat, long, alt, time
        gps_data = mp.Array('d', [0.0] * 4)
        main()
    except Exception as e:
        GPIO.output(BUZZ_PIN, GPIO.HIGH)
        time.sleep(2)
        GPIO.output(BUZZ_PIN, GPIO.LOW)
        GPIO.cleanup()
        raise(e)
        #os.system("sudo shutdown -h now")
