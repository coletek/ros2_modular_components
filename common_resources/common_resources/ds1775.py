from .gpio_singleton import GPIOSingleton

class DS1775:
    
    BUS = 1
    ADDRESS = 0x48
    REG_TEMP = 0x00

    def __init__(self, bus = BUS, address = ADDRESS):
        self._gpio = GPIOSingleton()
        self._fd = self._gpio.i2c_open(bus, address)

    def close(self):
        self._gpio.i2c_close(self._fd)
        
    def read(self):
        return float(self._gpio.i2c_read_byte_data(self._fd, self.REG_TEMP))
