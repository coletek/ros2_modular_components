import time
import yaml
import random
import sys
import tty
import termios
import select
from functools import partial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature

from common_resources.onewire_thermocouples import OneWireThermocouples

class SensorOnewire(Node):

    _ports = {}
    _pubs = {}
    _timer = {}
    
    def __init__(self):
        super().__init__('sensor_onewire')

        #self.declare_parameter('count', 8)
        #self.declare_parameter('ids', [ "3b-00000018370f", "3b-000000183730", "3b-0000001838b8", "3b-0000001838b6", "3b-000000183731", "3b-0000001838b4", "3b-0000001838bb", "3b-000000184195" ])
        #self.declare_parameter('types', [ "temperature", "temperature", "temperature", "temperature", "temperature", "temperature", "temperature", "temperature" ])
        #self.declare_parameter('freqs', [ 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 ])
        #self.declare_parameter('pubs', [ '/sensor_onewire/id0', '/sensor_onewire/id1', '/sensor_onewire/id2', '/sensor_onewire/id3', '/sensor_onewire/id4', '/sensor_onewire/id5', '/sensor_onewire/id6', '/sensor_onewire/id7' ])
        
        self.declare_parameter('count', 4)
        self.declare_parameter('ids', [ "id0", "id1", "id2", "id3" ])
        self.declare_parameter('types', [ "temperature", "temperature", "temperature", "temperature" ])
        self.declare_parameter('freqs', [ 0.5, 0.5, 0.5, 0.5 ])
        self.declare_parameter('pubs', [ '/sensor_onewire/id0', '/sensor_onewire/id1', '/sensor_onewire/id2', '/sensor_onewire/id3' ])

        self._onewire = OneWireThermocouples()
        
        for i in range(self.get_parameter("count").value):
            pub = self.get_parameter('pubs').value[i]
            self._ports[pub] = {
                "id": self.get_parameter('ids').value[i],
                "type": self.get_parameter('types').value[i],
                "freq": self.get_parameter('freqs').value[i],
            }
            self.get_logger().info("Created pub '%s'" % pub)
            if self.get_parameter('types').value[i] == "temperature":
                self.get_logger().info('created onewire temperature pub (%s)' % pub)
                self._pubs[pub] = self.create_publisher(Temperature, pub, 10)
            else:
                self.get_logger().info("unsupported onewire sensor type '%s'" % self.get_parameter('types').value[i])       
            self._timer[pub] = self.create_timer(1.0/self.get_parameter('freqs').value[i], partial(self.callback, name=pub))
            i += 1

        self.get_logger().info('Started')

    def callback(self, name):
        if name in self._ports:
            if self._ports[name]['type'] == "temperature":
                celsius = self._onewire.get_temperature(self._ports[name]["id"])
                if celsius > 0:
                    msg = Temperature()
                    #msg.temperature = random.uniform(0, 150.0) # sim
                    msg.temperature = celsius 
                    self._pubs[name].publish(msg)
                    self.get_logger().info('published onewire temperature (%s): %fdegC' % (name, celsius))
                else:
                    self.get_logger().info('error reading temp (%s): %fdegC' % (name, celsius))
            else:
                self.get_logger().info("unsupported onewire sensor type '%s'" % self._ports[name]['type'])
        else:
            self.get_logger().info("topic name '%s' not found" % name)

def non_blocking_getch():
    """Non-blocking version of getch()"""
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None
                    
def main(args=None):
    rclpy.init(args=args)
    node = SensorOnewire()
    active = True
    while rclpy.ok():
        key = non_blocking_getch()
        if key == 's':
            active = not active
        if active:
            rclpy.spin_once(node)
    node.destroy_onewire()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
