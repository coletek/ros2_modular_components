import random
import time
from .gpio_singleton import GPIOSingleton

class DR600:

    SETUP_MESSAGES = [
        "set RS 43\r\n",
        "set RA 43\r\n", 
        "set UN 3\r\n", 
        "set LO 1\r\n", 
        "set HI 331\r\n", 
        "set SP 331\r\n", 
        "set SF 0\r\n", 
        "set ST 99\r\n", 
        "set MO 1026\r\n", 
        "set MD 5\r\n", 
        "set IO 0\r\n", 
        "set HT 0\r\n", 
        "set BN 5\r\n", 
        "set TA 30\r\n", 
        "set TR 33667\r\n", 
        "set CY 500\r\n"
    ]
    
    _speed = 0
    _timestamp = 0
    _serial_data = []
    _is_completed = False
    
    def __init__(self, tty = "/dev/ttyS0", baud = 115200):
        self._gpio = GPIOSingleton()
        self._fd = self._gpio.serial_open(tty, baud)
                
    def setup(self):
        for m in self.SETUP_MESSAGES:
            self._gpio.serial_write(self._fd, m)
            time.sleep(1)
    
    def read(self):
        (b, d) = self._gpio.serial_read(self._fd, 1)
        if b > 0:
            data = list(d)
            for c in data:
                if chr(c) == '\n' or chr(c) == '\r':
                    speed = "".join(self._serial_data)
                    print ("%f DR600:tick(): speed='%s'" % (time.time(), speed))
                    if speed != "" and speed != '\n' and speed != '\r':
                        try:
                            if speed == "-000":
                                speed = "000"
                            self._speed = float(speed) / 3.6 # convert kph to m/s
                            self._timestamp = time.time()
                            self._is_completed = True
                            print ("speed=%f timestamp=%f" % \
                                   (self._speed, self._timestamp))
                        except ValueError:
                            print ("error parsing %s" % speed)
                            pass
                        self._serial_data = []
                    else:
                        self._is_completed = False
                else:
                    self._serial_data.append(chr(c))
                    self._is_completed = False
            return True
        else:
            return False
        
    def get_speed(self):
        return (self._timestamp, self._speed)
