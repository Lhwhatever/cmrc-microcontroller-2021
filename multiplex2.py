import time
import board
import busio
import smbus2
import adafruit_bno055
import adafruit_mpl3115a2
import altimeter

RPI_BUS_NUM = 1
MULTIPLEXER_ADDR = 0x70

I2C_CH = [1 << i for i in range(8)]

bus = smbus2.SMBus(RPI_BUS_NUM)
i2c = busio.I2C(board.SCL, board.SDA)

sensors = {}

def setup():
    bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[2])
    sensors['imu'] = adafruit_bno055.BNO055_I2C(i2c)
    bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[1])
    sensors['alti1'] = altimeter.Altimeter(i2c)
    bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[6])
    sensors['alti2'] = altimeter.Altimeter(i2c)
    bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[5])
    sensors['alti3'] = altimeter.Altimeter(i2c)


def loop():
    bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[2])
    print(sensors['imu'].acceleration)

    bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[1])
    print(sensors['alti1'].pressure)
    
    bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[6])
    print(sensors['alti2'].pressure)
    
    bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[5])
    print(sensors['alti3'].pressure)

setup()
while True:
    t0 =time.time()
    loop()
    print(time.time()-t0)
