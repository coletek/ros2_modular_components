import wx
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import *
from sensor_msgs.msg import *
from custom_messages.msg import *

from py_hardware.proxies.v4l import ProxyV4L

class GCamBox:
    def __init__(self):
        self.motor_speed = 0.0
        self.light_brightness = 0.0
        return

    def set_motor_speed(self, motor_speed):
        self.motor_speed = motor_speed

    def set_light_brightness(self, light_brightness):
        self.light_brightness = light_brightness

    def get_motor_speed(self):
        return self.motor_speed

    def get_light_brightness(self):
        return self.light_brightness
        
class View(wx.Frame):
    def __init__(self, parent, title, model, node):
        super(View, self).__init__(parent, title=title, size=(1000, 1000))

        self.model = model
        self.node = node

        self.panel = wx.Panel(self)

        self.image = wx.StaticBitmap(self.panel, size=(640,480))

        self.motor_label = wx.StaticText(self.panel, label="Motor Speed: 0.0 %")
        self.slider_motor = wx.Slider(self.panel, value=0, minValue=0, maxValue=100, style=wx.SL_HORIZONTAL)
        self.slider_motor.Bind(wx.EVT_SLIDER, self.on_slider_motor_changed)

        self.light_label = wx.StaticText(self.panel, label="Light Brightness: 0.0 %")
        self.slider_light = wx.Slider(self.panel, value=0, minValue=0, maxValue=100, style=wx.SL_HORIZONTAL)
        self.slider_light.Bind(wx.EVT_SLIDER, self.on_slider_light_changed)
        
        self.button = wx.Button(self.panel, label="Save")
        self.button.Bind(wx.EVT_BUTTON, self.on_button_clicked)
        
        sizer = wx.BoxSizer(wx.VERTICAL)

        sizer.Add(self.image, 0, wx.ALL | wx.CENTER, 5)

        sizer.Add(self.motor_label, 0, wx.ALL, 5)
        sizer.Add(self.slider_motor, 0, wx.ALL | wx.EXPAND, 5)

        sizer.Add(self.light_label, 0, wx.ALL, 5)
        sizer.Add(self.slider_light, 0, wx.ALL | wx.EXPAND, 5)

        sizer.Add(self.button, 0, wx.ALL | wx.CENTER, 5)

        self.panel.SetSizer(sizer)
        
    def on_slider_motor_changed(self, event):
        motor_speed_percent = self.slider_motor.GetValue()
        motor_speed = motor_speed_percent / 100.0
        self.node.get_logger().info('Changed slider motor value "%f"' % motor_speed)
        self.model.set_motor_speed(motor_speed)
        self.set_motor_text("Motor Speed: %.0f%%" % motor_speed_percent)

    def on_slider_light_changed(self, event):
        light_brightness_percent = self.slider_light.GetValue()
        light_brightness = light_brightness_percent / 100.0
        self.node.get_logger().info('Changed slider light value "%f"' % light_brightness)
        self.model.set_light_brightness(light_brightness)
        self.set_light_text("Light Brightness: %.0f%%" % light_brightness_percent)
        
    def on_button_clicked(self, event):
        motor_speed = self.model.get_motor_speed()
        msg = PWMMessage()
        msg.duty_cycle = motor_speed
        self.pub_motor.publish(msg)

        light_brightness = self.model.get_light_brightness()
        msg = PWMMessage()
        msg.duty_cycle = light_brightness
        self.pub_light.publish(msg)
        
    def set_pub_motor(self, pub):
        self.pub_motor = pub

    def set_pub_light(self, pub):
        self.pub_light = pub
        
    def set_motor_text(self, text):
        self.motor_label.SetLabel(text)

    def set_light_text(self, text):
        self.light_label.SetLabel(text)

        
class Controller(Node):
    def __init__(self):
        super().__init__("gcambox")
        self.model = GCamBox()

        self.app = wx.App()

        self.sub_image_sensor = ProxyV4L('rgb8')

        self.pub_motor = self.create_publisher(PWMMessage, '/device_motor', 10)
        self.pub_light = self.create_publisher(PWMMessage, "/device_light", 10)
        
        self.view = View(None, title="GUI", model=self.model, node=self)
        self.view.Show()
        
        rate = 5.0 # fps
        self.timer_camera = wx.Timer(self.app)
        self.app.Bind(wx.EVT_TIMER, self.update_camera, self.timer_camera)
        self.timer_camera.Start(int(1.0/rate * 1000))

        rate = 1
        self.timer_gui = wx.Timer(self.app)
        self.app.Bind(wx.EVT_TIMER, self.update_gui, self.timer_gui)
        self.timer_gui.Start(int(1.0/rate * 1000))
        
    def update_nodes(self, event):
        rclpy.spin_once(self)

    def update_camera(self, event):
        #self.get_logger().info("update_camera")
        self.sub_image_sensor.spin()

        # Update the image on the GUI
        image = self.sub_image_sensor.get_image()
        bitmap = wx.Bitmap.FromBuffer(image.shape[1], image.shape[0], image)
        self.view.image.SetBitmap(bitmap)
        self.view.panel.Layout()

    def update_gui(self, event):
        motor_speed = self.model.get_motor_speed()
        slider_motor_value = int(motor_speed * 100)
        if self.view.slider_motor.GetValue() != slider_motor_value:
            self.view.slider_motor.SetValue(slider_motor_value)

        light_brightness = self.model.get_light_brightness()
        slider_light_value = int(light_brightness * 100)
        if self.view.slider_light.GetValue() != slider_light_value:
            self.view.slider_light.SetValue(slider_light_value)

            
    def run(self):
        print ("run")
        self.app.MainLoop()
        #self.app.Yield() 

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
