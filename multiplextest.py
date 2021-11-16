import time
import board
import busio
import smbus2
import adafruit_bno055
import adafruit_mpl3115a2

RPI_BUS_NUM = 1
MULTIPLEXER_ADDR = 0x70

I2C_CH = [1 << i for i in range(8)]

bus = smbus2.SMBus(RPI_BUS_NUM)
i2c = busio.I2C(board.SCL, board.SDA)

sensors = {}

def setup():
    bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[7])
    sensors['imu'] = adafruit_bno055.BNO055_I2C(i2c)
    bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[6])
    sensors['alti1'] = adafruit_mpl3115a2.MPL3115A2(i2c)
    bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[5])
    sensors['alti2'] = adafruit_mpl3115a2.MPL3115A2(i2c)
    bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[4])
    sensors['alti3'] = adafruit_mpl3115a2.MPL3115A2(i2c)


def loop():
    #t0 = time.time()
    bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[7])
    #print("TIME: ", time.time()-t0)
    #t0 = time.time()
    print(sensors['imu'].acceleration)
    
    #print("TIME: ", time.time()-t0)
    bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[6])
    
    
    t0 = time.time()
    print(sensors['alti1'].pressure)
    print("TIME1: ", time.time()-t0)
    
    bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[5])
    
    t0 = time.time()
    print(sensors['alti2'].pressure)
    print("TIME2: ", time.time()-t0)
    
    bus.write_byte(MULTIPLEXER_ADDR, I2C_CH[4])
    
    t0 = time.time()
    print(sensors['alti3'].pressure)
    print("TIME3: ", time.time()-t0)

setup()
while True:
    #t0 = time.time()
    loop()
    #print(time.time()-t0)
