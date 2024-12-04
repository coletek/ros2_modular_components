from spidev import SpiDev

class MCP3008:
    def __init__(self, bus, device, speed = 1000000):
        self.spi = SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = speed
    
    def close(self):
        self.spi.close()
        
    def read(self, channel = 0):
        adc = self.spi.xfer2([1, (8 + channel) << 4, 0])
        data = ((adc[1] & 3) << 8) + adc[2]
        return data
