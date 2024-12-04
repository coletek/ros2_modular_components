import cv2

class V4L:

    def __init__(self, device = "/dev/video", width = -1, height = -1, fps = -1):
        self._fd = cv2.VideoCapture(device, cv2.CAP_V4L)
        self.set_properties(width, height, fps)

    def set_properties(self, width = -1, height = -1, fps = -1):
        if width > 0:
            self._fd.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height > 0:
            self._fd.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps > 0:
            self._fd.set(cv2.CAP_PROP_FPS, fps)
            
    def close(self):
        self._fd.release()
        
    def read(self):
        if not self._fd.isOpened():
            return -1, None
        else:
            return self._fd.read()

    def get_width(self):
        return self._fd.get(cv2.CAP_PROP_FRAME_WIDTH)

    def get_height(self):
        return self._fd.get(cv2.CAP_PROP_FRAME_HEIGHT)

    def get_fps(self):
        return self._fd.get(cv2.CAP_PROP_FPS)
    
    def save(self, filename):
        cv2.imwrite(filename, self._frame)

    def show(self, frame_name = "Frame"):
        cv2.imshow(frame_name, self._frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
