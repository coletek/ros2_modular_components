from functools import partial
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data

class ProxyV4L(Node):

    _image = {}
    
    def __init__(self, encoding="bgr8"):
        super().__init__('proxy_v4l')

        self._encoding = encoding
        self.declare_parameter('count', 1)
        self.declare_parameter('types', [ 'webcam' ])
        self.declare_parameter('pubs', [ '/sensor_webcam' ])

        for i in range(self.get_parameter("count").value):
            pub = self.get_parameter('pubs').value[i]

            # FIFO
            self.subscription = self.create_subscription(Image, pub, partial(self.callback, name=pub), qos_profile_sensor_data)
            self.bridge = CvBridge()
            self.subscription = self.create_subscription(Image, pub, partial(self.callback, name=pub), QoSProfile(depth=1))

            # FILO
            #self.subscription = self.create_subscription(Image, pub, partial(self.callback, name=pub, -1)

        self.get_logger().info("init()")
            
    def callback(self, msg, name  = "/sensor_webcam"):
        self._image[name] = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self._encoding)

    def spin(self):
        rclpy.spin_once(self)
        
    def get_image(self, name = "/sensor_webcam"):
        return self._image[name]
        
def main(args=None):
    rclpy.init(args=args)
    node = ProxyV4L()
    #rclpy.spin(node)    
    while rclpy.ok():
        node.spin()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
