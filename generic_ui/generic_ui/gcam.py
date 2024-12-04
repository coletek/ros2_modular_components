import wx
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import *
from sensor_msgs.msg import *
from custom_messages.msg import *

from py_hardware.proxies.v4l import ProxyV4L

class Model:
    _controller = None
    _image = None
    def __init__(self, controller):
        self._controller = controller
    def set_image(self, value):
        self._image = value
    def get_image(self):
        return self._image
    
class View(wx.Frame):

    _time_previous = 0.0
    _screen_count = 0
    
    def __init__(self, parent, title, controller):
        super(View, self).__init__(parent, title=title, size=(800, 600))

        # Double buffering is a technique used to reduce flickering
        # during graphical updates. It involves drawing the content
        # off-screen first and then swapping it to the display in a
        # single operation. While this helps to smooth out rendering
        # on more powerful machines, it can introduce unwanted side
        # effects on systems with limited resources, like the
        # Raspberry Pi. This Works for when we just have a image (and option 1 below),
        # but not when we have image+widgets - so we need option 2 below
        #self.SetDoubleBuffered(False)
        
        self._controller = controller

        panel = wx.Panel(self)

        # Add a button to the UI
        self.button = wx.Button(panel, label="Test Button", pos=(10, 10))
        self.button.Bind(wx.EVT_BUTTON, self.on_button_click)
        
        # option 1 - typical - for image only
        #self.image = wx.StaticBitmap(panel, size=(640,480))
        #self.Bind(wx.EVT_PAINT, self.on_paint)

        # option 2 - required if image + widgets
        self.image = wx.StaticBitmap(panel, size=(640,480))
        self._timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_paint, self._timer)
        self._fps = 40 # this needs to be above the config fps
        self._timer.Start(1000 // self._fps)

        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.image, 0, wx.ALL | wx.CENTER, 5)
        sizer.Add(self.button, 0, wx.ALL | wx.CENTER, 5)
        
        panel.SetSizer(sizer)

    def on_button_click(self, event):
        wx.MessageBox('Button clicked', 'Info', wx.OK | wx.ICON_INFORMATION)

    def on_paint(self, event):
        
        t = time.time()
        dt = t - self._time_previous
        if self._screen_count > 10:
            self._controller.get_logger().warn("screen hz=%.2f" % (1.0/dt*10.0))
            self._screen_count = 0
            self._time_previous = t
        self._screen_count += 1

        # draw image
        self._controller.set_image()
        image = self._controller.get_image()
        if image is not None:
            bitmap = wx.Bitmap.FromBuffer(image.shape[1], image.shape[0], image)
            self.image.SetBitmap(bitmap)
        self.Layout()
        
        self.Refresh()
        #wx.Yield()  # Ensure the buffer is fully flushed before moving on.
        
class Controller(Node):
    def __init__(self):
        super().__init__("gui")
        self.app = wx.App()

        self.sub_image_sensor = ProxyV4L('rgb8')

        self._model = Model(self)

        self.view = View(None, title="GUI", controller=self)
        self.view.Show()
        
    def set_image(self):
        self.sub_image_sensor.spin()
        image = self.sub_image_sensor.get_image()
        self._model.set_image(image)
        
    def get_image(self):
        return self._model.get_image()
        
    def run(self):
        self.app.MainLoop()

    def cleanup(self):
        self.view.Destroy()
        self.destroy_node()
        

def main(args=None):
    rclpy.init(args=None)
    node = Controller()
    node.run()
    node.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
