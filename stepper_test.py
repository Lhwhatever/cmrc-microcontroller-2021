import time
import board
import adafruit_bno055
import busio
import stepper

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

myStep = stepper.Stepper(23, 13, 15, 19, 21)
myStep.start_motor()
myStep.set_resolution('1/2')

myStep.move_motor(400, 1, 400)