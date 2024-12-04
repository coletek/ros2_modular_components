import time
import datetime
import math
import random

from functools import partial
import rclpy
from rclpy.node import Node
from common_resources.dr600 import DR600
from custom_messages.msg import SpeedMessage

class SensorSerial(Node):

    _ports = {}
    _serial = {}
    _pubs = {}
    _timer = {}
    _timestamp_previous = 0.0

    _is_sim = False
    
    def __init__(self):
        super().__init__('sensor_serial')

        self.declare_parameter('count', 1)
        self.declare_parameter('types', [ 'speed' ])
        self.declare_parameter('ttys', [ "/dev/ttyS0" ])
        self.declare_parameter('bauds', [ 115200 ])
        self.declare_parameter('pubs', [ '/sensor_speed' ])
        
        for i in range(self.get_parameter("count").value):
            pub = self.get_parameter('pubs').value[i]
            type = self.get_parameter("types").value[i]
            tty = self.get_parameter('ttys').value[i]
            baud = self.get_parameter('bauds').value[i]
            self._ports[pub] = {
                "type": type,
                "tty": tty,
                "baud": baud,
            }
            self.get_logger().info("Created pub '%s'" % pub)            
            if type == "speed":
                self._serial[pub] = DR600(tty, baud)
                self._pubs[pub] = self.create_publisher(SpeedMessage, pub, 10)

            freq = math.ceil(self.get_parameter('bauds').value[i] / 8.0 / 4.0 / 1000.0) * 1000.0 # assuming 4 bytes, and 8 bits per byte
            freq = 30
            self.get_logger().info("set polling freq to %.0fHz" % freq)
            if self._is_sim:
                self._timer[pub] = self.create_timer(1.0/freq, partial(self.callback_sim, name=pub))
            else:
                self._timer[pub] = self.create_timer(1.0/freq, partial(self.callback, name=pub))
            i += 1

        self.get_logger().info('Started')

    def callback(self, name):
        if name in self._ports:
            if self._ports[name]['type'] == "speed":
                ret = self._serial[name].read()
                if ret and self._serial[name]._is_completed:
                    (timestamp, meters_per_second) = self._serial[name].get_speed()
                    msg = SpeedMessage()
                    msg.meters_per_second = meters_per_second
                    self._pubs[name].publish(msg)
                    self.get_logger().info('published serial speed (%s): %fm/s at %s (%f)' % (name, meters_per_second, datetime.datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d %H:%M:%S'), timestamp))
            else:
                self.get_logger().info("unsupported serial sensor type '%s'" % self._ports[name]['type'])
        else:
            self.get_logger().info("topic name '%s' not found" % name)

    def callback_sim(self, name):
        if name in self._ports:
            if self._ports[name]['type'] == "speed":
                timestamp = time.time()
                meters_per_second = random.uniform(-70.0, 70.0)

                msg = SpeedMessage()
                msg.meters_per_second = meters_per_second
                self._pubs[name].publish(msg)
                self.get_logger().info('published serial speed (%s): %fm/s at %s (%f)' % (name, meters_per_second, datetime.datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d %H:%M:%S'), timestamp))
            else:
                self.get_logger().info("unsupported serial sensor type '%s'" % self._ports[name]['type'])
        else:
            self.get_logger().info("topic name '%s' not found" % name)
            
def main(args=None):
    rclpy.init(args=args)
    node = SensorSerial()
    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
