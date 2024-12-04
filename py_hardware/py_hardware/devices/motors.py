import os
import getch
import yaml
import random
from functools import partial
import rclpy
from rclpy.node import Node
from custom_messages.msg import PWMMessage

from common_resources.gpio_singleton import GPIOSingleton

class DeviceMotors(Node):

    _count = 0
    _motors = {}

    def __init__(self):
        super().__init__('device_motors')

        #self.declare_parameter('count', 1)
        #self.declare_parameter('pins_pwm', [ 17 ])
        #self.declare_parameter('pins_dir', [ 16 ])
        #self.declare_parameter('freqs', [ 25000 ])
        #self.declare_parameter('subs', ['/device_motor'])
        
        self.declare_parameter('count', 4)
        self.declare_parameter('pins_pwm', [ 18, 20, 22, 24 ])
        self.declare_parameter('pins_dir', [ 19, 21, 23, 25 ])
        self.declare_parameter('freqs', [ 25000, 25000, 25000, 25000 ])
        self.declare_parameter('speed_min', [ 0, 0, 0, 0 ])
        self.declare_parameter('speed_max', [ 255, 255, 255, 255 ])
        self.declare_parameter('subs', ['/device_motor', '/device_motor/ch1', '/device_motor/ch2', '/device_motor/ch3'])

        self._gpio = GPIOSingleton()
        
        for i in range(self.get_parameter("count").value):
            sub = self.get_parameter('subs').value[i]
            pin_pwm = self.get_parameter('pins_pwm').value[i]
            pin_dir = self.get_parameter('pins_dir').value[i]
            freq = self.get_parameter('freqs').value[i]
            speed_min = self.get_parameter('speed_min').value[i]
            speed_max = self.get_parameter('speed_max').value[i]
            self._motors[sub] = {
                "pin_pwm": pin_pwm,
                "pin_dir": pin_dir,
                "freq": freq,
                "speed_min": speed_min,
                "speed_max": speed_max
            }
            self.get_logger().info("Created sub '%s'" % sub)
            self.create_subscription(PWMMessage, sub, partial(self.callback, name=sub), 10)
            i += 1

            self._gpio.set_mode(pin_dir, "output")
            #self._gpio.set_mode(pin_pwm, "output")
            self._gpio.set_PWM_dutycycle(pin_pwm, 0)
            self._gpio.set_PWM_frequency(pin_pwm, freq)
            
        self.get_logger().info('Started')

    def callback(self, msg, name):
        if msg.duty_cycle < 0:
            self._gpio.write(self._motors[name]["pin_dir"], True)
        else:
            self._gpio.write(self._motors[name]["pin_dir"], False)
        speed = int(abs(msg.duty_cycle *  self._motors[name]["speed_max"]))
        if speed > self._motors[name]["speed_max"]:
            speed = self._motors[name]["speed_max"]
        elif speed < self._motors[name]["speed_min"] and speed != 0:
            speed = self._motors[name]["speed_min"]
        self._gpio.set_PWM_dutycycle(self._motors[name]["pin_pwm"], speed)

        self.get_logger().info("Received '%s' %f%% - setting pins (pwm=%d dir=%d) with duty_cycle=%f" %
                               (name, msg.duty_cycle * 100, self._motors[name]['pin_pwm'],
                                self._motors[name]['pin_dir'], speed))

def main(args=None):
    rclpy.init(args=args)

    node = DeviceMotors()

    rclpy.spin(node)
    #while rclpy.ok():
    #    key = getch.getch()
    #    if key == 'i':
    #        node.get_logger().info('Keyboard-based motor')
    #        msg = PWMMessage()
    #        msg.duty_cycle = random.uniform(-1.0, 1.0)
    #        node.callback(msg, "/device_motor")
                    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
