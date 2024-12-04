import getch
import yaml
import random
import pyaudio
from functools import partial
import rclpy
from rclpy.node import Node
from custom_messages.srv import BuzzerServiceMessage

from common_resources.sounds import Sounds
from common_resources.gpio_singleton import GPIOSingleton

class DeviceBuzzers(Node):

    def __init__(self):
        super().__init__('device_buzzers')

        # TODO: update to support more then one buzzer, and perhaps rename
        # this node to speaker or audio and we have different types of
        # audio output device
        
        self.declare_parameter('mode', 'pi')
        self.declare_parameter('pin', 12)
        self.declare_parameter("srv", "/device_buzzer/mode" )

        if self.get_parameter("mode").value == "pyaudio":
            self._pyaudio = pyaudio.PyAudio()
        else:
            self._gpio = GPIOSingleton()
            self._gpio.set_mode(self.get_parameter("pin").value, "output")
        
        self.sounds = Sounds()

        self.create_service(BuzzerServiceMessage, self.get_parameter('srv').value, self.callback_mode) # replaced with sounds
            
        self.get_logger().info('Started')

    def callback_mode(self, request, response):

        if self.sounds.is_sound_available(request.sound_name):
            self.play(request.sound_name)
            response.success = True
            response.message = ""
        else:
            response.success = False
            response.message = "Sound not available"
            
        self.get_logger().info('Incoming sound request (%s): %s (%r)' % (request.sound_name, response.message, response.success))
        return response

    def play(self, sound_name):

        self.get_logger().info("Playing sound (%s)" % sound_name)

        duration, samples, combined_tones = self.sounds.get_waveform(sound_name)

        if self.get_parameter("mode").value == "pyaudio":
            
            # Create a PyAudio stream and play the tones
            stream = self._pyaudio.open(format=pyaudio.paFloat32, channels=1, rate=self.sounds.sample_rate, output=True, frames_per_buffer=4096)
            #stream.write(combined_tones.tobytes())
            stream.write(combined_tones.astype(np.float32).tobytes())
            stream.close()
            
        else:

            for tone in self.sounds.sounds[sound_name]:
                frequency = 0
                duration = 0
                volume = 0
                for key, val in tone.items():
                    if key == "freq":
                        frequency = val
                    if key == "duration":
                        duration = val
                    if key == "volume":
                        volume = val
                self.get_logger().info("buzzer tone freq=%f duration=%f volume=%f" % (frequency, duration, volume))
                if frequency > 0:
                    self._gpio.set_PWM_frequency(self.get_parameter("pin").value, abs(frequency))
                    self._gpio.set_PWM_dutycycle(self.get_parameter("pin").value, 128)
                else:
                    self._gpio.set_PWM_dutycycle(self.get_parameter("pin").value, 0)
                time.sleep(duration)
            self._gpio.set_PWM_dutycycle(self.get_parameter("pin").value, 0)

        return True
            
def main(args=None):
    rclpy.init(args=args)

    node = DeviceBuzzers()

    rclpy.spin(node)
    #while rclpy.ok():
    #    key = getch.getch()
    #    if key == 'i':
    #        node.get_logger().info('Keyboard-based buzzer')
    #        msg = PWMMessage()
    #        msg.duty_cycle = random.uniform(-1.0, 1.0)
    #        node.callback(msg, "/device_buzzer")
                    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
