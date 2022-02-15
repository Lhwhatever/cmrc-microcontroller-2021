import stepper
import time
import board
import adafruit_bno055
import busio

kP = 10
kI = 0.5
kD = 0

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)
motor = stepper.Stepper(23, 13, 15, 19, 21)
motor.set_resolution("1/2")
motor.start_motor()

t0 = time.time()
i = 0
lastError = 0
while True:
    # get the gravity vector from the board
    grav = sensor.gravity
    t = time.time()

    # find the error by:
    # - removing all z component (which we can't fix)
    # - normalizing the vec
    # - dotting it with [0, -1, 0]

    gravLen = (grav[0]**2 + grav[1]**2)**.5
    normalGravY = grav[1]/gravLen
    error = -normalGravY

    # now to run the PI controller
    # yummm pie
    p = error
    i = error*(t - t0) + i*.8
    d = (error - lastError)/(t - t0)

    controlOut = kP*p + kI*i + kD*d

    # motor time bois!!
    # each step should take 0.05 sec
    motorSteps = abs(controlOut)
    stepDir = controlOut//motorSteps
    stepFreq = motorSteps/0.05

    motor.move_motor(motorSteps, stepDir, stepFreq)

    t0 = t
    lastError = error