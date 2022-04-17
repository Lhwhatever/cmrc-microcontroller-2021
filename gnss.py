import serial
import time
import RPi.GPIO as GPIO

class Gnss:
    def __init__(self, sel:int, rst:int, exi:int, pls:int):
        self.sel = sel
        self.rst = rst
        self.exi = exi
        self.pls = pls
        GPIO.setwarnings(False) 
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.sel, GPIO.OUT)
        GPIO.setup(self.rst, GPIO.OUT)
        GPIO.setup(self.exi, GPIO.IN)
        GPIO.setup(self.pls, GPIO.IN)  

        self.ser = serial.Serial(
            port='/dev/serial0',
            baudrate = 38400,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=10
        )  

        GPIO.output(self.sel, GPIO.HIGH)
    
    def wakeup(self):
        GPIO.output(self.sel, GPIO.HIGH)
        GPIO.output(self.rst, GPIO.HIGH)
        time.sleep(.001)
        #GPIO.output(self.rst, GPIO.LOW)
        time.sleep(.002)
        GPIO.output(self.rst, GPIO.HIGH)
        time.sleep(.001)
        #GPIO.output(self.rst, GPIO.LOW)

    def set_sel(self, state:bool):
        GPIO.output(self.sel, state)

    def set_rst(self, state:bool):
        GPIO.output(self.rst, state)
    
    def get_exi(self) -> bool:
        return GPIO.input(self.exi)

    def get_pls(self) -> bool:
        return GPIO.input(self.pls)

    def uart_write(self, data:str):
        self.ser.write(data)
    
    def uart_read(self):
        return self.ser.readline()

mygnss = Gnss(18, 23, 24, 25)
mygnss.wakeup()
while True:
    try:
        read = mygnss.uart_read()
        print(read.decode(), end="")
    except:
        mygnss = Gnss(18, 23, 24, 25)
        #mygnss.wakeup()
        #GPIO.cleanup()
        print("failed")
        
GPIO.cleanup()
print("")

