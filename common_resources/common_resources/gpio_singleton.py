'''
To use this class, simply create an instance of it and call its methods:

gpio = GPIOSingleton()
gpio.write(4, 1)  # Sets pin 4 to high
value = await gpio.read(4)  # Reads value from pin 4 asynchronously
gpio.set_PWM_dutycycle(18, 128)  # Sets PWM output on pin 18 to 50% duty cycle
gpio.write_byte(0x20, 0x01)  # Writes byte 0x01 to I2C address 0x20
value = gpio.read_byte(0x20)  # Reads byte from I2C address 0x20
'''

import random
import time

try:
    import pigpio
except:
    pass

class GPIOSingleton:
    _instance = None
    _pigpio = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(GPIOSingleton, cls).__new__(cls)
            try:
                cls._pigpio = pigpio.pi()
                print("pigpio initialized", flush = True)
            except:
                print("pigpio not available, using simulation", flush = True)
        return cls._instance

    def get_pigpio(self):
        return self._pigpio

    # -------------------------------------------------------------------------
    # DIO
    # -------------------------------------------------------------------------

    def set_mode(self, pin, mode):
        if self._pigpio:
            if mode == "output":
                self._pigpio.set_mode(pin, pigpio.OUTPUT)
            elif mode == "input":
                self._pigpio.set_mode(pin, pigpio.INPUT)
        else:
            # simulation code for set_mode
            print("set_mode(%d, %s)" % (pin, mode))

    def read(self, pin):
        if self._pigpio:
            return self._pigpio.read(pin)
        else:
            # Simulation code for async digital read
            time.sleep(0.1)
            return random.randint(0, 1)

    def write(self, pin, value):
        if self._pigpio:
            self._pigpio.write(pin, value)
        else:
            # Simulation code for digital output
            print("write(%d, %d)" % (pin, value), flush = True)

    # -------------------------------------------------------------------------
    # PWM
    # -------------------------------------------------------------------------

    def set_PWM_frequency(self, pin, value):
        if self._pigpio:
            self._pigpio.set_PWM_frequency(pin, value)
        else:
            # Simulation code for PWM output
            print("set_PWM_frequency(%d, %d)" % (pin, value), flush = True)

    def set_PWM_dutycycle(self, pin, value):
        if self._pigpio:
            self._pigpio.set_PWM_dutycycle(pin, value)
        else:
            # Simulation code for PWM output
            print("set_PWM_dutycycle(%d, %d)" % (pin, value), flush = True)

    # -------------------------------------------------------------------------
    # Serial/UART
    # -------------------------------------------------------------------------

    def serial_open(self, tty, baud):
        if self._pigpio:
            return self._pigpio.serial_open(tty, baud)
        else:
            # Simulation code for Serial/UART open
            print("serial_open(%s, %d)" % (tty, baud), flush = True)
            return -1

    def serial_close(self, handle):
        if self._pigpio:
            self._pigpio.serial_close(handle)
        else:
            # Simulation code for Serial/UART open
            print("serial_close()", flush = True)

    def serial_write(self, handle, data):
        if self._pigpio:
            self._pigpio.serial_write(handle, data)
        else:
            # Simulation code for Serial/UART write
            print("serial_write() data=%s (0x%x)" % (data, data), flush = True)
        
    def serial_read(self, handle, count):
        # returns (number_of_bytes_read, data_bytearray)
        if self._pigpio:
            return self._pigpio.serial_read(handle, count)
        else:
            # Simulation code for Serial/UART read
            return (0, bytearray())

    # -------------------------------------------------------------------------
    # I2C
    # -------------------------------------------------------------------------

    # NEEDS TESTING

    def i2c_open(self, bus, address):
        if self._pigpio:
            return self._pigpio.i2c_open(bus, address)
        else:
            # Simulation code for I2C open
            print("i2c_open(%d, 0x%02x)" % (bus, address), flush = True)
            return -1
        
    def i2c_close(self, handle):
        if self._pigio:
            self._pigpio.i2c_close(handle)
        else:
            # Simulation code for I2C close
            print("i2c_close(handle)", flush = True)
            
    def i2c_write_byte_data(self, handle, reg, value):
        if self._pigpio:
            self._pigpio.i2c_write_byte_data(handle, reg, value)
        else:
            # Simulation code for I2C write data
            print("i2c_write_byte(handle, 0x%02x, 0x%02x)" % (reg, value), flush = True)

    def i2c_read_byte_data(self, handle, reg):
        if self._pigpio:
            return self._pigpio.i2c_read_byte_data(handle, reg)
        else:
            # Simulation code for async byte read
            time.sleep(0.1)
            return random.randrange(0, 255, 1)
        
    # -------------------------------------------------------------------------
    # SPI
    # -------------------------------------------------------------------------

    # NEEDS TESTING, BUT I THINK THE BELOW spi_open comment means the
    # speed setting doesn't work, so using SpiDev for now
    
    def spi_open(self, channel, baud, flags = 0):

        # the default settings within pigio is 125kHz speed and mode
        # 0, which means that the clock signal is idle low (CPOL=0)
        # and data is sampled on the leading (first) edge of the clock
        # signal (CPHA=0).
        #
        # ChatGPT says to can change the SPI speed to 1Mhz via the
        # flags: flags = pigpio.SPI_SPEED | 1000000, however I think
        # the baud argument is the speed

        if self._pigpio:
            if flags > 0:
                return self._pigpio.spi_open(channel, baud, flags)
            else:
                return self._pigpio.spi_open(channel, baud)
        else:
            # Simulation code for SPI open
            print("spi_open(%d, %d)" % (channel, baud), flush = True)
            return -1

    def spi_close(self, handle):
        if self._pigpio:
            self._pigpio.spi_close(handle)
        else:
            # Simulation code for SPI close
            print("spi_close()", flush = True)

    def spi_read(self, handle, count):
        if self._pigpio:
            return self._pigpio.spi_read(handle, count)
        else:
            # Simulation code for SPI read
            print("spi_read()", flush = True)

    def spi_write(self, handle, data):
        if self._pigpio:
            return self._pigpio.spi_write(handle, data)
        else:
            # Simulation code for SPI write
            print("spi_write(0x%x)" % data, flush = True)

    def spi_xfer(self, handle, data):
        if self._pigpio:
            return self._pigpio.spi_xfer(handle, data)
        else:
            # Simulation code for SPI xfer
            print("spi_xfer(0x%x)" % data, flush = True)

    # -------------------------------------------------------------------------
    # one-wire
    # -------------------------------------------------------------------------

    # TODO - see onewire_thermocouples.py for now
