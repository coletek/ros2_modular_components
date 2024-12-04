import getch
import yaml
import rclpy
from rclpy.node import Node
from custom_messages.msg import InterruptMessage

class SensorInterrupts(Node):

    _count = 0
    _interrupts = {}
    _pubs = {}
    
    def __init__(self):
        super().__init__('sensor_interrupts')

        self.declare_parameter('count', 4)
        self.declare_parameter('pins', [ 18, 20, 22, 24 ])
        self.declare_parameter('edges', [ "rising", "falling", "both", "rising" ])
        self.declare_parameter('pubs', [ '/sensor_interrupt', '/sensor_interrupt/ch1', '/sensor_interrupt/ch2', '/sensor_interrupt/ch3' ])

        for i in range(self.get_parameter("count").value):
            pub = self.get_parameter('pubs').value[i]
            self._interrupts[pub] = {
                "pin": self.get_parameter('pins').value[i],
                "edge": self.get_parameter('edges').value[i],
            }
            self.get_logger().info("Created pub '%s'" % pub)
            self._pubs[pub] = self.create_publisher(InterruptMessage, pub, 10)
            i += 1
        
        self.get_logger().info('Started')

    def callback(self, name, edge):
        if name in self._interrupts:
            if edge == self._interrupts[name]['edge']:
                self._pubs[name].publish(InterruptMessage())
                self.get_logger().info("Interrupt '%s' pin %d" % (name, self._interrupts[name]['pin']))
            else:
                self.get_logger().info("Interrupt edge '%s' doesn't match trigger (%s)" % (edge, self._interrupts[name]['edge']))
        else:
            self.get_logger().info("Interrupt '%s' not found" % name)

def main(args=None):
    rclpy.init(args=args)

    node = SensorInterrupts()
    
    while rclpy.ok():
        key = getch.getch()
        if key == '0':
            node.get_logger().info('Keyboard-based interrupt')
            node.callback('/sensor_interrupt', "rising")        
        if key == '1':
            node.get_logger().info('Keyboard-based interrupt')
            node.callback('/sensor_interrupt/ch1', "rising")
        if key == '2':
            node.get_logger().info('Keyboard-based interrupt')
            node.callback('/sensor_interrupt/ch2', "rising")        
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
