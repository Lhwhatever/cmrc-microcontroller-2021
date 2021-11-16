import time
import board
import adafruit_bno055
import busio

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

while True:
    acc = sensor.acceleration
    print("Acceleration: {0:0.3f}, {1:0.3f}, {2:0.3f}".format(acc[0], acc[1], acc[2]))
    
    grav = sensor.gravity
    print("Gravity: {0:0.3f}, {1:0.3f}, {2:0.3f}".format(grav[0], grav[1], grav[2]))
    
    gyro = sensor.gyro
    print("Gyroscope: {0:0.3f}, {1:0.3f}, {2:0.3f}".format(gyro[0], gyro[1], gyro[2]))
    
    temperature = sensor.temperature
    print("Temp: {0:0.3f} deg C".format(temperature))
    
    time.sleep(1.0)
