import time
import board
import adafruit_mpl3115a2

i2c = board.I2C()
sensor = adafruit_mpl3115a2.MPL3115A2(i2c)

sensor.sealevel_pressure = 102250

while True:
    t0 = time.time()
    pressure = sensor.pressure
    print("Pressure: {0:0.3f} pascals".format(pressure))
    print("Time: ", time.time() - t0)
    
    altitude = sensor.altitude
    print("Altitude: {0:0.3f} meters".format(altitude))
    
    temperature = sensor.temperature
    print("Temp: {0:0.3f} deg C".format(temperature))
