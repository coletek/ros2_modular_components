import time
import math
import yaml
import random
import sys
import tty
import termios
import select
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from custom_messages.msg import *

from common_resources.mcp3008 import MCP3008

class SensorMCP3008(Node):

    _ports = {}
    _pubs = {}

    LM335_SCALE = 0.01 # 10mV/K
    KELVIN_TO_DEGREE = -273.15
    
    def __init__(self):
        super().__init__('sensor_mcp3008')

        '''
        self.declare_parameter("vref", 4.096)
        self.declare_parameter("bits", 1023)
        self.declare_parameter("ntc_beta", 3950.0)
        self.declare_parameter("ntc_resistance", 10e3)
        self.declare_parameter('spi_bus', 0)
        self.declare_parameter('spi_device', 1)
        self.declare_parameter('spi_baud', 1000000)
        self.declare_parameter("freq", 1.0)
        self.declare_parameter('ch_count', 8)
        self.declare_parameter('chs', [ 0, 1, 2, 3, 4, 5, 6, 7 ])
        self.declare_parameter('types', [ 'percentage', 'percentage', 'percentage', 'percentage', 'percentage', 'percentage', 'lm335dt', 'ntc' ])
        self.declare_parameter('pubs', [ '/sensor_mcp3008/ch0', '/sensor_mcp3008/ch1', '/sensor_mcp3008/ch2', '/sensor_mcp3008/ch3',
                                         '/sensor_mcp3008/ch4', '/sensor_mcp3008/ch5', '/sensor_mcp3008/ch6', '/sensor_mcp3008/ch7' ])
        '''
        
        self.declare_parameter("vref", 4.096)
        self.declare_parameter("bits", 1023)
        self.declare_parameter("ntc_beta", 3950.0)
        self.declare_parameter("ntc_resistance", 10e3)
        self.declare_parameter('spi_bus', 0)
        self.declare_parameter('spi_device', 1)
        self.declare_parameter('spi_baud', 1000000)
        self.declare_parameter("freq", 1.0)
        self.declare_parameter('ch_count', 8)
        self.declare_parameter('chs', [ 0, 1, 2, 3, 4, 5, 6, 7 ])
        self.declare_parameter('types', [ 'percentage', 'percentage', 'percentage', 'percentage', 'percentage', 'percentage', 'temperature', 'temperature' ])
        self.declare_parameter('pubs', [ '/sensor_mcp3008/ch0', '/sensor_mcp3008/ch1', '/sensor_mcp3008/ch2', '/sensor_mcp3008/ch3',
                                         '/sensor_mcp3008/ch4', '/sensor_mcp3008/ch5', '/sensor_mcp3008/ch6', '/sensor_mcp3008/ch7' ])
        
        self._mcp3008 = MCP3008(self.get_parameter("spi_bus").value,
                                self.get_parameter("spi_device").value,
                                self.get_parameter("spi_baud").value)
        
        for i in range(self.get_parameter("ch_count").value):
            pub = self.get_parameter('pubs').value[i]
            ch = self.get_parameter('chs').value[i]
            type_id = self.get_parameter('types').value[i]
            self._ports[pub] = {
                "ch": ch,
                "type": type_id,
            }
            self.get_logger().info("Created pub '%s'" % pub)

            if type_id == "percentage":
                self._pubs[pub] = self.create_publisher(PercentageMessage, pub, 10)
            elif type_id == "ntc" or type_id == "lm335dt":
                self._pubs[pub] = self.create_publisher(Temperature, pub, 10)
            else:
                self.get_logger().info('no type found (%s)' % type_id)
            i += 1
            
        self._timer = self.create_timer(1.0/self.get_parameter("freq").value, self.callback)

        self.get_logger().info('Started')
        
    def callback(self):
        # TODO: replace this simulation with real query to GPIOSingleton
        for key, val in self._ports.items():

            #v = random.uniform(0, self.get_parameter("vref").value) # sim
            v = self._mcp3008.read(val["ch"]) / self.get_parameter("bits").value * self.get_parameter("vref").value

            if val["type"] == "percentage":
                msg = PercentageMessage()
                msg.percent = v / self.get_parameter("vref").value
                self._pubs[key].publish(msg)
                self.get_logger().info("published '%s' : %f%% (%fV)" % (key, msg.percent * 100.0, v))
            elif val["type"] == "ntc":
                msg = Temperature()
                msg.temperature = self.calc_temperature_ntc(v)
                msg.variance = 0.0
                self._pubs[key].publish(msg)
                self.get_logger().info("published '%s' : %fdegC (%fV)" % (key, msg.temperature, v))
            elif val["type"] == "lm335dt":
                msg = Temperature()
                msg.temperature = self.calc_temperature_lm335dt(v)
                msg.variance = 0.0
                self._pubs[key].publish(msg)
                self.get_logger().info("published '%s' : %fdegC (%fV)" % (key, msg.temperature, v))
            else:
                self.get_logger().info("type (%s) not implemented - got voltage=%fV" % (val["type"], v))

    def calc_temperature_ntc(self, volts):
        beta = self.get_parameter("ntc_beta").value
        Rref = self.get_parameter("ntc_resistance").value
        Vref = self.get_parameter("vref").value
        Rload = 4300
        Rnew = Rload * volts / (Vref - volts)
        Rinf = Rref * math.exp(- beta / 298.15)
        if Rinf == 0:
            self.get_logger().info("Rinf is 0 - Rref=%f beta=%f" % (Rref, beta))
            return -1
        T = beta / math.log(Rnew / Rinf) + self.KELVIN_TO_DEGREE
        return T

    def calc_temperature_lm335dt(self, volts):
        return volts / self.LM335_SCALE + self.KELVIN_TO_DEGREE
    
def non_blocking_getch():
    """Non-blocking version of getch()"""
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None
                    
def main(args=None):
    rclpy.init(args=args)
    node = SensorMCP3008()
    active = True
    while rclpy.ok():
        key = non_blocking_getch()
        if key == 's':
            active = not active
        if active:
            rclpy.spin_once(node)
    node.destroy_mcp3008()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
