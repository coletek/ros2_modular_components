import random

from functools import partial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import *
from custom_messages.msg import VoltageMessage
from common_resources.tsl2591 import TSL2591
from common_resources.ds1775 import DS1775

class SensorI2C(Node):

    _ports = {}
    _i2c = {}
    _pubs = {}
    _timer = {}
    
    def __init__(self):
        super().__init__('sensor_i2c')

        self.declare_parameter('count', 3)
        self.declare_parameter('types', [ 'temperature', 'light', 'battery' ])
        self.declare_parameter('buses', [ 1, 1, 1 ])
        self.declare_parameter('addresses', [ 0x48, 0x29, 0xAB ])
        self.declare_parameter('freqs', [ 1.0, 1.0, 1.0 ])
        self.declare_parameter('pubs', [ '/sensor_temperature', '/sensor_light', '/sensor_battery' ])
        
        for i in range(self.get_parameter("count").value):
            pub = self.get_parameter('pubs').value[i]
            type = self.get_parameter("types").value[i]
            bus = self.get_parameter('buses').value[i]
            address = self.get_parameter('addresses').value[i]
            freq = self.get_parameter('freqs').value[i]
            self._ports[pub] = {
                "type": type,
                "bus": bus,
                "address": address,
                "freq": freq
            }
            self.get_logger().info("Created pub '%s'" % pub)            
            if type == "temperature":
                self._i2c[pub] = DS1775(bus, address)
                self._pubs[pub] = self.create_publisher(Temperature, pub, 10)
            elif type == "light":
                self._i2c[pub] = TSL2591(bus, address)
                self._pubs[pub] = self.create_publisher(Illuminance, pub, 10)                
            elif type == "battery":
                #self._i2c[pub] = DS1775(bus, address) # TODO
                self._pubs[pub] = self.create_publisher(VoltageMessage, pub, 10)                
            self._timer[pub] = self.create_timer(1.0/self.get_parameter('freqs').value[i], partial(self.callback, name=pub))            
            i += 1

        self.get_logger().info('Started')

    def callback(self, name):
        if name in self._ports:
            if self._ports[name]['type'] == "temperature":
                celsius = self._i2c[name].read() # sim option in gpio_singleton
                if celsius > 0:
                    msg = Temperature()
                    #msg.temperature = 
                    msg.temperature = celsius 
                    self._pubs[name].publish(msg)
                    self.get_logger().info('published i2c temperature (%s): %fdegC' % (name, celsius))
                else:
                    self.get_logger().info('error reading temperature (%s): %fdegC' % (name, celsius))
            elif self._ports[name]['type'] == "light":
                lux = self._i2c[name].read() # sim option in gpio_singleton
                if lux > 0:
                    msg = Illuminance()
                    msg.illuminance = lux 
                    self._pubs[name].publish(msg)
                    self.get_logger().info('published i2c illuminance (%s): %flux' % (name, lux))
                else:
                    self.get_logger().info('error reading illuminance (%s): %flux' % (name, lux))
            elif self._ports[name]['type'] == "battery":
                #volts = self._i2c[name].read() # TODO
                volts = random.uniform(0.0, 12.0) # sim
                if volts > 0:
                    msg = VoltageMessage()
                    msg.volts = volts
                    self._pubs[name].publish(msg)
                    self.get_logger().info('published i2c battery (%s): %fV' % (name, volts))
                else:
                    self.get_logger().info('error reading battery (%s): %fV' % (name, volts))
            else:
                self.get_logger().info("unsupported i2c sensor type '%s'" % self._ports[name]['type'])
        else:
            self.get_logger().info("topic name '%s' not found" % name)

def main(args=None):
    rclpy.init(args=args)
    node = SensorI2C()
    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
