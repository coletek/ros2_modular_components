import time
from .gpio_singleton import GPIOSingleton

class TSL2591:
    
    BUS = 1
    ADDRESS = 0x29

    REG_COMMAND_BIT = 0xa0

    REG_REGISTER_ENABLE = 0x00
    REG_REGISTER_CONTROL = 0x01
    REG_REGISTER_DEVICE_ID = 0x12
    REG_REGISTER_CHAN0_LOW = 0x14
    REG_REGISTER_CHAN1_LOW = 0x16
    
    REG_ENABLE_POWEROFF = 0x00
    REG_ENABLE_POWERON = 0x01
    REG_ENABLE_AEN = 0x02
    REG_ENABLE_AIEN = 0x10
    REG_ENABLE_NPIEN = 0x80
    
    REG_INTEGRATIONTIME_100MS = 0x00
    REG_INTEGRATIONTIME_200MS = 0x01
    REG_INTEGRATIONTIME_300MS = 0x02
    REG_INTEGRATIONTIME_400MS = 0x03
    REG_INTEGRATIONTIME_500MS = 0x04
    REG_INTEGRATIONTIME_600MS = 0x05
    
    REG_GAIN_LOW = 0x00 # 1x
    REG_GAIN_MED = 0x10 # 25x
    REG_GAIN_HIGH = 0x20 # 428x
    REG_GAIN_MAX = 0x30 # 9876x

    REG_LUX_DF = 408 # lux cooefficient

    def __init__(self, bus = BUS, address = ADDRESS):
        self._gpio = GPIOSingleton()
        self._fd = self._gpio.i2c_open(bus, address)
        self.setup()

    def close(self):
        self._gpio.i2c_close(self._fd)

    def setup(self):
        self._integration = self.REG_INTEGRATIONTIME_100MS
        self._gain = self.REG_GAIN_MED
  
        # check ID
        id = self._gpio.i2c_read_byte_data(self._fd,
                                           self.REG_COMMAND_BIT |
                                           self.REG_REGISTER_DEVICE_ID)
        if id != 0x50:
            #print ("error: wrong id")
            return -1

        # enable
        self._gpio.i2c_write_byte_data(self._fd,
                                       self.REG_COMMAND_BIT |
                                       self.REG_REGISTER_ENABLE,
                                       self.REG_ENABLE_POWERON |
                                       self.REG_ENABLE_AEN |
                                       self.REG_ENABLE_AIEN |
                                       self.REG_ENABLE_NPIEN)

        # set timing and gain
        self._gpio.i2c_write_byte_data(self._fd,
                                       self.REG_COMMAND_BIT |
                                       self.REG_REGISTER_CONTROL,
                                       self._integration | self._gain)


        # disable
        self._gpio.i2c_write_byte_data(self._fd,
                                       self.REG_COMMAND_BIT |
                                       self.REG_REGISTER_ENABLE,
                                       self.REG_ENABLE_POWEROFF)

        return 0

    def read(self):

        # enable
        self._gpio.i2c_write_byte_data(self._fd,
                                       self.REG_COMMAND_BIT |
                                       self.REG_REGISTER_ENABLE,
                                       self.REG_ENABLE_POWERON |
                                       self.REG_ENABLE_AEN |
                                       self.REG_ENABLE_AIEN |
                                       self.REG_ENABLE_NPIEN)
    
        # wait for adc
        time.sleep(1)
    
        y = self._gpio.i2c_read_byte_data(self._fd,
                                          self.REG_COMMAND_BIT |
                                          self.REG_REGISTER_CHAN0_LOW)
        x = self._gpio.i2c_read_byte_data(self._fd,
                                          self.REG_COMMAND_BIT |
                                          self.REG_REGISTER_CHAN1_LOW)

        x <<= 16
        x |= y
        lum = x

        #print ("full_luminosity=%d" % lum)

        ir = lum >> 16
        full = lum & 0xFFFF

        #print ("full=%d ir=%d" % (full, ir))
  
        # calculate actual lux value
        # full, ir
  
        if full == 0xffff | ir == 0xffff:
            #print ("error: overflow")
            return -1

        if self._integration == self.REG_INTEGRATIONTIME_100MS:
            atime = 100
        elif self._integration == self.REG_INTEGRATIONTIME_200MS:
            atime = 200
        elif self._integration == self.REG_INTEGRATIONTIME_300MS:
            atime = 300
        elif self._integration == self.REG_INTEGRATIONTIME_400MS:
            atime = 400
        elif self._integration == self.REG_INTEGRATIONTIME_500MS:
            atime = 500
        elif self._integration == self.REG_INTEGRATIONTIME_600MS:
            atime = 600
        else:
            atime = 100

        if self._gain == self.REG_GAIN_LOW:
            again = 1
        elif self._gain == self.REG_GAIN_MED:
            again = 25
        elif self._gain == self.REG_GAIN_HIGH:
            again = 428
        elif self._gain == self.REG_GAIN_MAX:
            again = 9876
        else:
            again = 1

        cpl = (atime * again) / self.REG_LUX_DF;

        if full > 0 and cpl > 0:
            lux = (full - ir) * (1.0 - ir / full) / cpl;
        else:
            lux = 0.0
       
        # disable
        self._gpio.i2c_write_byte_data(self._fd,
                                       self.REG_COMMAND_BIT |
                                       self.REG_REGISTER_ENABLE,
                                       self.REG_ENABLE_POWEROFF)

        return lux
