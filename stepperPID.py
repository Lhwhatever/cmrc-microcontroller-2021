import time

import stepper
from CircularBuffer import CircularBuffer

kP = 10
kI = 0.5

def stepperInit():
    # runs init code for steppers, should be run in the descent_2_setup
    motor = stepper.Stepper(23, 13, 15, 19, 21)
    motor.set_resolution("1/2")
    motor.start_motor()


def stepperPID(turn_rate, data_avg: CircularBuffer,
    rt_data: CircularBuffer):
    # specify ideal turn rate in steps (0 results in no motion)

    # get the true turn rate (based on rolling avg to remove noise)
    # and compare with setpoint to get error
    rt_turn_rate = data_avg[-1]['imu']['gyr'] - data_avg[-2]['imu']['gyr']
    error = rt_turn_rate - turn_rate

    # pi time
    p = error
    i = 0
    for index in range(-1, max(-10, -len(data_avg)+2)):
        rt_turn_rate = (data_avg[index]['imu']['gyr'] 
            - data_avg[index]['imu']['gyr'])
        i += rt_turn_rate - turn_rate
    
    controlOut = kP*p + kI*i

    # motor time bois!!
    # each step should take 0.05 sec
    motorSteps = abs(controlOut)
    stepDir = controlOut//motorSteps
    stepFreq = motorSteps/0.05

    motor.move_motor(motorSteps, stepDir, stepFreq)