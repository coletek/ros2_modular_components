from functools import partial
import rclpy
from rclpy.node import Node
from std_msgs.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from common_resources.v4l import V4L

class SensorV4L(Node):

    _ports = {}
    _v4ls = {}
    _bridges = {}
    _pubs = {}
    _timer = {}
    
    def __init__(self):
        super().__init__('sensor_v4l')

        self.declare_parameter('count', 1)
        self.declare_parameter('types', [ 'webcam' ])
        self.declare_parameter('devices', [ '/dev/video0' ])
        self.declare_parameter('widths', [ 640 ])
        self.declare_parameter('heights', [ 480 ])
        self.declare_parameter('freqs', [ 20.0 ])
        self.declare_parameter('encodings', [ 'bgr8' ])
        self.declare_parameter('pubs', [ '/sensor_webcam' ])
        
        for i in range(self.get_parameter("count").value):
            pub = self.get_parameter('pubs').value[i]
            type = self.get_parameter("types").value[i]
            device = self.get_parameter('devices').value[i]
            width = self.get_parameter('widths').value[i]
            height = self.get_parameter('heights').value[i]
            freq = self.get_parameter('freqs').value[i]
            encoding = self.get_parameter('encodings').value[i]
            self._ports[pub] = {
                "type": type,
                "device": device,
                "width": width,
                "height": height,
                "freq": freq,
                "encoding": encoding
            }
            self.get_logger().info("Created pub '%s'" % pub)            
            if type == "webcam":
                self._v4ls[pub] = V4L(device, width, height, int(freq))
                self.get_logger().warn("camera setup with width=%d height=%d fps=%d" %
                                       (self._v4ls[pub].get_width(), self._v4ls[pub].get_height(), self._v4ls[pub].get_fps()))
                self._bridges[pub] = CvBridge()
                self._pubs[pub] = self.create_publisher(Image, pub, 10)
            self._timer[pub] = self.create_timer(1.0/self.get_parameter('freqs').value[i], partial(self.callback, name=pub))            
            i += 1

        self.get_logger().info('Started')

    def callback(self, name):
        if name in self._ports:
            if self._ports[name]['type'] == "webcam":
                ret, frame = self._v4ls[name].read()
                if ret > 0:
                    msg = self._bridges[name].cv2_to_imgmsg(frame, encoding=self._ports[name]['encoding'])
                    self._pubs[name].publish(msg)
                    self.get_logger().info('published v4l webcam (%s)' % name)
                else:
                    self.get_logger().info('error v4l webcam (%s)' % name)
            else:
                self.get_logger().info("unsupported v4l webcam sensor type '%s'" % self._ports[name]['type'])
        else:
            self.get_logger().info("topic name '%s' not found" % name)

def main(args=None):
    rclpy.init(args=args)
    node = SensorV4L()
    rclpy.spin(node)
    #while rclpy.ok():
    #    rclpy.spin_once(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
