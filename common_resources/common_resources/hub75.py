import time
import random
import socket
from PIL import Image

HOST, PORT = "localhost", 9999

class HUB75:
    _instance = None
    _sock = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(HUB75, cls).__new__(cls)
            try:
                print("matrix connecting....", flush = True)
                cls._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                cls._sock.connect((HOST, PORT))  
                print("matrix connected", flush = True)
            except:
                print("matrix not available, using simulation", flush = True)
        return cls._instance

    def reconnect(cls):
        try:
            if cls._sock:
                cls._sock.close()
                cls._sock = None
                print("Previous connection closed", flush=True)
                
            print("matrix connecting....", flush=True)
            cls._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            cls._sock.connect((HOST, PORT))
            print("matrix connected", flush=True)
        except Exception as e:
            cls._sock = None
            print(f"matrix not available, using simulation. Error: {e}", flush=True)

    def send_command(self, command):
        try:
            self._sock.sendall((command + "\n").encode())
        except:
            print("send_command() failed - reconnecting")
            self.reconnect()
            pass

    def clear(self):
        print("clear()")
        if self._sock:
            self.send_command("CLEAR")

    def set_brightness(self, brightness):
        print("set_brightness(%d)" % brightness)
        if self._sock:
            self.send_command("SET_BRIGHTNESS %d" % brightness)

    def set_pixel(self,  x, y, r, g, b):
        print("set_pixel(%d, %d, %d, %d, %d)" % (x, y, r, g, b))
        if self._sock:
            self.send_command("SET_PIXEL %d %d %d %d %d" % (x, y, r, g, b))

    def set_fill(self,  r, g, b):
        print("set_fill(%d, %d, %d)" % (x, y, r, g, b))
        if self._sock:
            self.send_command("SET_FILL %d %d %d" % (r, g, b))
            
    def set_image_from_filepath(self, filepath):
        print("set_image_from_filepath(%s)" % filepath)
        if self._sock:
            self.send_command("SET_IMAGE_FROM_FILEPATH %s" % filepath)

    def set_text(self, x, y, fontpath, text):
        print("set_text(%d, %d, %s, %s)" % (x, y, fontpath, text))
        if self._sock:
            self.send_command("SET_TEXT %d %d %s %s" % (x, y, fontpath, text))
            
    '''
    def setup_canvas(self):
        print("setup_canvas()")
        if self._sock:
            self.send_command("SETUP_CANVAS" % brightness)
        
    def set_scrolling_text(self, y, text, font_path = "./rpi-rgb-led-matrix/fonts/7x13.bdf"):
        print("set_scrolling_text(%d, %s, %s)" % (y, text, font_path))
        if self._sock:
            self.send_command("SETUP_CANVAS" % brightness)
                
    def write_canvas(self):
        print("write_canvas()")
        if self._sock:
            self.send_command("WRITE_CANVAS")
    '''
