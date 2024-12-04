import getch
import random
from functools import partial
import rclpy
from rclpy.node import Node
from custom_messages.msg import PWMMessage
from common_resources.gpio_singleton import GPIOSingleton

class DeviceFans(Node):

    _count = 0
    _fans = {}
    
    def __init__(self):
        super().__init__('device_fans')

        self.declare_parameter('count', 4)
        self.declare_parameter('pins', [ 13, 20, 22, 24 ])
        self.declare_parameter('freqs', [ 25000, 25000, 25000, 25000 ])
        self.declare_parameter('subs', [ '/device_fan', '/device_fan/ch1', '/device_fan/ch2', '/device_fan/ch3' ])

        self._gpio = GPIOSingleton()
         
        for i in range(self.get_parameter("count").value):
            sub = self.get_parameter('subs').value[i]
            pin = self.get_parameter('pins').value[i]
            freq = self.get_parameter('freqs').value[i]
            self._fans[sub] = {
                "pin": pin,
                "freq": freq
            }
            self.get_logger().info("Created sub '%s'" % sub)
            self.create_subscription(PWMMessage, sub, partial(self.callback, name=sub), 10)
            i += 1

            self._gpio.set_mode(pin, "output")
            self._gpio.set_PWM_frequency(pin, freq)
            self._gpio.set_PWM_dutycycle(pin, 0)
            
        self.get_logger().info('Started')

    def callback(self, msg, name):
        self.get_logger().info("Received '%s' %f%% - setting pin=%d" %
                               (name, msg.duty_cycle, self._fans[name]['pin']))
        self._gpio.set_PWM_dutycycle(self._fans[name]["pin"], int(abs(msg.duty_cycle) * 255))
        
def main(args=None):
    rclpy.init(args=args)
    node = DeviceFans()
    rclpy.spin(node)
    #while rclpy.ok():
    #    key = getch.getch()
    #    if key == 'i':
    #        node.get_logger().info('Keyboard-based fan')
    #        msg = PWMMessage()
    #        msg.duty_cycle = random.uniform(-1.0, 1.0)
    #        node.callback(msg, "/device_fan")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
