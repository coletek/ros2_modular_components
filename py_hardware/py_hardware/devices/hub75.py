import getch
import random
from functools import partial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from custom_messages.msg import *
from common_resources.hub75 import HUB75

class DeviceHUB75(Node):

    def __init__(self):
        super().__init__('device_hub75')

        self.declare_parameter('subs', [ '/device_display_clear', '/device_display_brightness',
                                         '/device_display_text', '/device_display_text_scrolling',
                                         '/device_display_image_from_filepath', '/device_display_image' ])

        self._hub75 = HUB75()
        
        i = 0
        for sub in self.get_parameter("subs").value:
            if i == 0:
                msg_type = DisplayClearMessage
            elif i == 1:
                msg_type = DisplayBrightnessMessage                
            elif i == 2:
                msg_type = DisplayTextMessage                
            elif i == 3:
                msg_type = DisplayTextScrollingMessage                
            elif i == 4:
                msg_type = DisplayImageFromFilepathMessage
            elif i == 5:
                msg_type = Image
            self.create_subscription(msg_type, sub, partial(self.callback, name=sub), 10)
            i += 1
        
        self.get_logger().info('Started')

    def callback(self, msg, name):
        self.get_logger().info('Started')

        if name == "/device_display_clear":
            self.get_logger().info("set display clear %r" % msg.is_clear)
            if msg.is_clear:
                self._hub75.clear()
        elif name == "/device_display_brightness":
            self.get_logger().info("set display brightness %d%%" % msg.percent)
            self._hub75.set_brightness(msg.percent)
        elif name == "/device_display_text":
            self.get_logger().info("set display text '%s' at [%d %d] using font=%s" % (msg.text, msg.x, msg.y, msg.fontpath))
            self._hub75.set_text(msg.x, msg.y, msg.fontpath, msg.text)
        #elif name == "/device_display_text_scrolling":
        #    self.get_logger().info("set display scrolling text '%s' at y=%d using font=%s" % (msg.scrolling_text, msg.y, msg.font_path))
        #    self._hub75.set_scrolling_text(msg.y, msg.txt, msg.font_path)
        elif name == "/device_display_image_from_filepath":
            self.get_logger().info("set display image from filepath")
            self._hub75.set_image_from_filepath(msg.filepath)
        #elif name == "/device_display_image":
        #    self.get_logger().info("set display image")
        #    self._hub75.set_image(msg.data)
        
def main(args=None):
    rclpy.init(args=args)
    node = DeviceHUB75()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
