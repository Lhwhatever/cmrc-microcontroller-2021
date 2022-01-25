# Carnegie Mellon Rocket Command, Device Driver for TMC2208 Motor Driver IC
# Chris Stange, <cjstange@andrew.cmu.edu> (December 26, 2021)

# THIS DEVICE DRIVER OPERATES THE TMC2208 IN LEGACY MODE
import RPi.GPIO as GPIO
import time

# Pin numbers are specified by their pin header number, not the GPIO number
GPIO.setmode(GPIO.BOARD)

class Stepper:
    def __init__(self, en_pin, res1_pin, res2_pin, dir_pin, step_pin):
        # In order to control the TMC2208 we need an Enable, 2 Resolution, a Direction, and a Step pin
        self.en_pin = en_pin # This is the EN pin on the TMC2208
        self.res1_pin = res1_pin # This is the MS1 pin on the TMC2208
        self.res2_pin = res2_pin # This is the MS2 pin on the TMC2208
        self.dir_pin = dir_pin # This is the DIR pin on the TMC2208
        self.step_pin = step_pin # This is the STEP pin on the TMC2208

        # Set all pins as outputs
        GPIO.setup(self.en_pin, GPIO.OUT)
        GPIO.setup(self.res1_pin, GPIO.OUT)
        GPIO.setup(self.res2_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)

        # Initialize motor as off
        self.enable = False
        GPIO.output(self.en_pin, 1)

        # Intialize resolution to 1/2
        self.resolution = (0,1)
        GPIO.output(self.res1_pin, self.resolution[0])
        GPIO.output(self.res2_pin, self.resolution[1])

        # Initialize direction as forward
        GPIO.output(self.dir_pin, 0)

        # Initialize no motor movement
        GPIO.output(self.step_pin, 0)

        # Resolution dictionary
        self.dict = {'1/2': (0,1),
                        '1/4': (1,0),
                        '1/8': (0,0),
                        '1/16': (1,1)}
        
    # Enables motor
    def start_motor(self):
        self.enable = True
        GPIO.output(self.en_pin, 0)

    # Disables motor
    def stop_motor(self):
        self.enable = False
        GPIO.output(self.en_pin, 1)

    # Sets resolution
    # Resolution can either be 1/2, 1/4, 1/8, or 1/16
    def set_resolution(self, resolution):
        if resolution in self.dict:
            self.resolution = self.dict[resolution]
            GPIO.output(self.res1_pin, self.resolution[0])
            GPIO.output(self.res2_pin, self.resolution[1])
        else:
            printf("Invalid resolution\nValid resolutions are '1/2', '1/4', '1/8', '1/16'")

    # Moves the stepper
    # Input the number of steps, the direction, and the frequency at which the steps should be made
    # Number of steps and frequency are ints
    # Direction is 0 for forward and 1 for backward
    # THIS FUNCTION WILL NOT DO ANYTHING IF start_motor() IS NOT CALLED FIRST
    def move_motor(self, step_count, step_direction, step_frequency):
        if self.enable:
            if (step_direction == 0) or (step_direction == 1):
                if isinstance(step_count, int) and isinstance(step_frequency, int):
                    step_period = 1//step_frequency
                    GPIO.output(self.dir_pin, step_direction)
                    for i in range(step_count):
                        GPIO.output(self.step_pin, 1)
                        time.sleep(step_period)
                        GPIO.output(self.step_pin, 0)
                else:
                    print("Invalid step count or frequency\nPlease ensure inputs are ints")
            else:
                print("Invalid motor direction\nPlease input 0 for forward\nPlease input 1 for backwards")
        else:
            print("Motor not enabled\nPlease call start_motor() first")
