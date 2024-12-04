import time
import getch
import yaml
import random
from functools import partial
import rclpy
from rclpy.node import Node
from custom_messages.msg import StateMessage

from common_resources.max7301atl import MAX7301ATL

class DeviceMAX7301ATL(Node):

    _ports = {}
    _pubs = {}
    
    def __init__(self):
        super().__init__('device_max7301atl')

        #self.declare_parameter('spi_bus', 0)
        #self.declare_parameter('spi_device', 0)
        #self.declare_parameter('spi_baud', 1000000)
        #self.declare_parameter('ch_count', 8)
        #self.declare_parameter('chs', [ 12, 9, 13, 10, 14, 11, 15, 16 ])
        #self.declare_parameter('modes', [ "output", "output", "output", "output", "output", "output", "output", "output" ])
        #self.declare_parameter('is_inverse_state', [ False, False, False, False, True, True, True, True ])
        #self.declare_parameter('topics', [ '/device_max7301atl/ch0', '/device_max7301atl/ch1', '/device_max7301atl/ch2', '/device_max7301atl/ch3',
        #                                   '/device_max7301atl/ch4', '/device_max7301atl/ch5', '/device_max7301atl/ch6', '/device_max7301atl/ch7' ])

        self.declare_parameter('spi_bus', 0)
        self.declare_parameter('spi_device', 0)
        self.declare_parameter('spi_baud', 1000000)
        self.declare_parameter('ch_count', 8)
        self.declare_parameter('chs', [ 0, 1, 2, 3, 4, 5, 6, 7 ])
        self.declare_parameter('modes', [ "output", "output", "output", "output", "input", "input", "input", "input" ])
        self.declare_parameter('is_inverse_state', [ False, False, False, False, True, True, True, True ])
        self.declare_parameter('topics', [ '/device_max7301atl/ch0', '/device_max7301atl/ch1', '/device_max7301atl/ch2', '/device_max7301atl/ch3',
                                           '/device_max7301atl/ch4', '/device_max7301atl/ch5', '/device_max7301atl/ch6', '/device_max7301atl/ch7' ])

        self._max7301 = MAX7301ATL(self.get_parameter("spi_bus").value,
                                   self.get_parameter("spi_device").value,
                                   self.get_parameter("spi_baud").value)
        self._max7301.set_all_io_output_and_low()
        #time.sleep(0.1)

        for i in range(self.get_parameter("ch_count").value):
            topic = self.get_parameter('topics').value[i]
            ch = self.get_parameter('chs').value[i]
            mode = self.get_parameter('modes').value[i]
            is_inverse_state = self.get_parameter('is_inverse_state').value[i]
            self._ports[topic] = {
                "ch": ch,
                "mode": mode,
                "is_inverse_state": is_inverse_state
            }
            if self.get_parameter('modes').value[i] == "output":
                self.get_logger().info("Created sub '%s'" % topic)
                self.create_subscription(StateMessage, topic, partial(self.callback, name=topic), 10)
            else:
                self.get_logger().info("Created pub '%s'" % topic)
                self._pubs[topic] = self.create_publisher(StateMessage, topic, 10)
            i += 1

            if mode == "output":
                self._max7301.set_pin_as_output(ch)
            else:
                self._max7301.set_pin_as_input(ch)
                
            if is_inverse_state:
                self._max7301.set_pin(ch, 1) # ensures all devices are off

        self.get_logger().info('Started')

    def callback(self, msg, name):
        self.get_logger().info("Received message '%s' %r - setting ch %d" % (name, msg.state, self._ports[name]['ch']))
        if self._ports[name]['is_inverse_state']:
            self._max7301.set_pin(self._ports[name]['ch'], int(not msg.state))
        else:
            self._max7301.set_pin(self._ports[name]['ch'], int(msg.state))

    def callback_input(self, msg, name):
        self.get_logger().info("Received message '%s' on ch %d: %d" % (name, self._ports[name]['ch'], msg.state))
        # TODO
        
def main(args=None):
    rclpy.init(args=args)

    node = DeviceMAX7301ATL()

    rclpy.spin(node)
    #while rclpy.ok():
    #    key = getch.getch()
    #    if key == 'i':
    #        node.get_logger().info('Keyboard-based max7301atl')
    #        msg = StateMessage()
    #        msg.state = random.uniform(0.0, 1.0)
    #        node.callback(msg)
                    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
