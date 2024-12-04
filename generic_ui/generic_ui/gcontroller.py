import wx
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import *
from sensor_msgs.msg import *
from custom_messages.msg import *

from py_hardware.proxies.v4l import ProxyV4L

class GController:
    def __init__(self):
        self.battery_sensor = 0.0
        self.temperature_sensor = 0.0
        self.light_sensor = 0.0
        self.radar_sensor = 0.0
        self.fan_speed = 0.0
        self.display_brightness = 0
        self.display_cavnas = ""

    def set_battery_sensor(self, value):
        self.battery_sensor = value

    def set_temperature_sensor(self, value):
        self.temperature_sensor = value
        
    def set_light_sensor(self, value):
        self.light_sensor = value

    def set_radar_sensor(self, value):
        self.radar_sensor = value
        
    def set_fan_speed(self, fan_speed):
        self.fan_speed = fan_speed

    def set_display_brightness(self, display_brightness):
        self.display_brightness = display_brightness

    def set_display_canvas(self, display_canvas):
        self.display_canvas = display_cavnas

    def get_battery_sensor(self):
        return self.battery_sensor

    def get_temperature_sensor(self):
        return self.temperature_sensor
        
    def get_light_sensor(self):
        return self.light_sensor

    def get_radar_sensor(self):
        return self.radar_sensor
    
    def get_fan_speed(self):
        return self.fan_speed

    def get_display_brightness(self):
        return self.display_brightness
    
class View(wx.Frame):
    def __init__(self, parent, title, model, node, is_camera_included):
        super(View, self).__init__(parent, title=title, size=(1000, 1000))

        self.model = model
        self.node = node
        self.is_camera_included = is_camera_included

        self.panel = wx.Panel(self)

        if self.is_camera_included:
            self.image = wx.StaticBitmap(self.panel, size=(640,480))

        self.battery_sensor_label = wx.StaticText(self.panel, label="Battery Sensor: 0.0 V")
        self.temperature_sensor_label = wx.StaticText(self.panel, label="Temperature Sensor: 0.0 K")
        self.light_sensor_label = wx.StaticText(self.panel, label="Light Sensor: 0.0 lux")
        self.radar_sensor_label = wx.StaticText(self.panel, label="Radar Sensor: 0.0 m/s")
        self.fan_label = wx.StaticText(self.panel, label="Fan Speed: 0.0 %")
        self.display_brightness_label = wx.StaticText(self.panel, label="Display Brightness: 0 %")

        self.slider_fan = wx.Slider(self.panel, value=0, minValue=0, maxValue=100, style=wx.SL_HORIZONTAL)
        self.slider_fan.Bind(wx.EVT_SLIDER, self.on_slider_fan_changed)

        self.slider_display_brightness = wx.Slider(self.panel, value=0, minValue=0, maxValue=100, style=wx.SL_HORIZONTAL)
        self.slider_display_brightness.Bind(wx.EVT_SLIDER, self.on_slider_display_brightness_changed)
        
        self.button = wx.Button(self.panel, label="Save")
        self.button.Bind(wx.EVT_BUTTON, self.on_button_clicked)

        sizer = wx.BoxSizer(wx.VERTICAL)
        if self.is_camera_included:
            sizer.Add(self.image, 0, wx.ALL | wx.CENTER, 5)
        sizer.Add(self.battery_sensor_label, 0, wx.ALL, 5)
        sizer.Add(self.temperature_sensor_label, 0, wx.ALL, 5)
        sizer.Add(self.light_sensor_label, 0, wx.ALL, 5)
        sizer.Add(self.radar_sensor_label, 0, wx.ALL, 5)
        sizer.Add(self.fan_label, 0, wx.ALL, 5)
        sizer.Add(self.slider_fan, 0, wx.ALL | wx.EXPAND, 5)
        sizer.Add(self.display_brightness_label, 0, wx.ALL, 5)
        sizer.Add(self.slider_display_brightness, 0, wx.ALL | wx.EXPAND, 5)
        sizer.Add(self.button, 0, wx.ALL | wx.CENTER, 5)

        self.panel.SetSizer(sizer)

    def on_slider_fan_changed(self, event):
        fan_speed_percent = self.slider_fan.GetValue()
        fan_speed = fan_speed_percent / 100.0
        self.node.get_logger().info('Changed slider_fan value "%f"' % fan_speed)
        self.model.set_fan_speed(fan_speed)
        self.set_fan_text("Fan Speed: %f%%" % fan_speed_percent)

    def on_slider_display_brightness_changed(self, event):
        display_brightness_percent = self.slider_display_brightness.GetValue()
        self.node.get_logger().info('Changed slider_display_brightness value "%d"' % display_brightness_percent)
        self.model.set_display_brightness(display_brightness_percent)
        self.set_display_brightness_text("Display Brightness: %d%%" % display_brightness_percent)
        
    def on_button_clicked(self, event):
        fan_speed = self.model.get_fan_speed()
        msg = PWMMessage()
        msg.duty_cycle = fan_speed
        self.pub_fan.publish(msg)

        display_brightness = self.model.get_display_brightness()
        msg = DisplayBrightnessMessage()
        msg.percent = display_brightness
        self.pub_display_brightness.publish(msg)

    def set_pub_fan(self, pub):
        self.pub_fan = pub

    def set_pub_display_brightness(self, pub):
        self.pub_display_brightness = pub
        
    def set_fan_text(self, text):
        self.fan_label.SetLabel(text)

    def set_display_brightness_text(self, text):
        self.display_brightness_label.SetLabel(text)

    def set_battery_sensor_text(self, text):
        self.battery_sensor_label.SetLabel(text)

    def set_temperature_sensor_text(self, text):
        self.temperature_sensor_label.SetLabel(text)
        
    def set_light_sensor_text(self, text):
        self.light_sensor_label.SetLabel(text)

    def set_radar_sensor_text(self, text):
        self.radar_sensor_label.SetLabel(text)
        
class Controller(Node):
    def __init__(self, is_camera_included):
        super().__init__("gui")
        self.is_camera_included = is_camera_included
        self.model = GController()

        self.app = wx.App()

        if self.is_camera_included:
            self.sub_image_sensor = ProxyV4L('rgb8')

        # FIFI
        ##qos_profile = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        ##qos_profile = rclpy.qos.qos_profile_sensor_data(depth=1)
        self.create_subscription(VoltageMessage, '/sensor_battery', self.callback_battery_sensor, QoSProfile(depth=1))
        self.create_subscription(Temperature, '/sensor_temperature', self.callback_temperature_sensor, QoSProfile(depth=1))
        self.create_subscription(Illuminance, '/sensor_light', self.callback_light_sensor, QoSProfile(depth=1))
        self.create_subscription(SpeedMessage, '/sensor_speed', self.callback_radar_sensor, QoSProfile(depth=1))

        
        # FILO
        #self.sub_light_sensor = self.create_subscription(
        #    Float32,
        #    'light_sensor/lux',
        #    self.light_sensor_callback,
        #    10)
        #self.sub_radar_sensor = self.create_subscription(
        #    Float32,
        #    'radar_sensor/speed',
        #    self.radar_sensor_callback,
        #    10)

        self.pub_fan = self.create_publisher(PWMMessage, '/device_fan', 10)


        self.pub_display_brightness = self.create_publisher(DisplayBrightnessMessage, "/device_display_brightness", 10)
        
        self.view = View(None, title="GUI", model=self.model, node=self, is_camera_included=self.is_camera_included)
        self.view.set_pub_fan(self.pub_fan)
        self.view.set_pub_display_brightness(self.pub_display_brightness)
        self.view.Show()

        rate = 30.0
        self.timer = wx.Timer(self.app)
        self.app.Bind(wx.EVT_TIMER, self.update_nodes, self.timer)
        self.timer.Start(int(1.0/rate * 1000))

        if self.is_camera_included:
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
        if self.is_camera_included:
            #self.get_logger().info("update_camera")
            self.sub_image_sensor.spin()
    
            # Update the image on the GUI
            image = self.sub_image_sensor.get_image()
            bitmap = wx.Bitmap.FromBuffer(image.shape[1], image.shape[0], image)
            self.view.image.SetBitmap(bitmap)
            self.view.panel.Layout()
        
    def update_gui(self, event):
        fan_speed = self.model.get_fan_speed()
        slider_fan_value = int(fan_speed * 100)
        if self.view.slider_fan.GetValue() != slider_fan_value:
            self.view.slider_fan.SetValue(slider_fan_value)

    def callback_battery_sensor(self, msg):
        self.model.set_battery_sensor(msg.volts)
        self.view.set_battery_sensor_text(f"Battery Sensor: {msg.volts} V")

    def callback_temperature_sensor(self, msg):
        self.model.set_temperature_sensor(msg.temperature)
        self.view.set_temperature_sensor_text(f"Temperature Sensor: {msg.temperature} degC")
            
    def callback_light_sensor(self, msg):
        self.model.set_light_sensor(msg.illuminance)
        self.view.set_light_sensor_text(f"Light Sensor: {msg.illuminance} lux")

    def callback_radar_sensor(self, msg):
        self.model.set_radar_sensor(msg.meters_per_second)
        self.view.set_radar_sensor_text(f"Radar Sensor: {msg.meters_per_second} m/s")
        
    def run(self):
        print ("run")
        self.app.MainLoop()
        #self.app.Yield() 

    def cleanup(self):
        self.timer.Stop()
        self.view.Destroy()
        self.destroy_node()
        

def main(args=None):
    rclpy.init(args=None)

    parser = argparse.ArgumentParser(description='GController')
    parser.add_argument("-c", '--include_camera', action='store_true', help='Include camera')
    
    args = parser.parse_args()
    is_camera_included = args.include_camera

    node = Controller(is_camera_included)
    node.run()
    node.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
