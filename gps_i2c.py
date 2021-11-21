from smbus2 import smbus2

smbus = smbus2.SMBus(1)
smbus.open(1)

I2C_ADDR = 0x42

smbus.write_i2c_block_data(
        I2C_ADDR,
        0xff,
        [
            0xFF,
            0xB5, 0x62,
            0x01, 0x07,
            0x00, 0x00,
            0x00, 0x00
        ]
        )

len1 = smbus.read_byte_data(I2C_ADDR, 0xFD)
len2 = smbus.read_byte_data(I2C_ADDR, 0xFE)

left = (len1 << 8) | len2

data = []
while left > 0:
    to_read = min(32, left)
    data += smbus.read_i2c_block_data(I2C_ADDR, 0xFF, to_read)
    left -= to_read

print(data)
