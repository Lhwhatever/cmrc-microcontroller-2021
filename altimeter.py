import adafruit_mpl3115a2 as mpl

class Altimeter(mpl.MPL3115A2):
    def __init__(self, i2c, *, address=mpl._MPL3115A2_ADDRESS): 
        super().__init__(i2c, address=address)
        self._ctrl_reg1 = mpl._MPL3115A2_CTRL_REG1_OS1 | mpl._MPL3115A2_CTRL_REG1_ALT
        self._write_u8(mpl._MPL3115A2_CTRL_REG1, self._ctrl_reg1)

