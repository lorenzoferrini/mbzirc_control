import faulthandler
faulthandler.enable()

from threading import Thread
import cv2
from ConfigParser import SafeConfigParser
import os.path


class CameraError(Exception):
   # Camera doesn't work properly
   pass

class VideoGet:
    """
    Class that continuously gets frames from a VideoCapture object
    with a dedicated thread.
    """
    
    def __init__(self, src=0):
        my_path = os.path.abspath(os.path.dirname(__file__))
        path = os.path.join(my_path, "config/configuration.ini")
        self.config = SafeConfigParser()
        self.config.read(path)
        self.stream = cv2.VideoCapture(src)
        self.stream.set(3, self.config.getint('video_get','width'))
        self.stream.set(4, self.config.getint('video_get','height'))
        print self.stream.get(3)
        print self.stream.get(4)            
        (self.grabbed, self.frame) = self.stream.read()
        if not self.grabbed:
            print src
            raise CameraError
        self.stopped = False

    def start(self):    
        Thread(target=self.get, args=()).start()
        return self

    def get(self):
        while not self.stopped:
            if not self.grabbed:
                self.stop()
            else:
                (self.grabbed, self.frame) = self.stream.read()

    def stop(self):
        self.stopped = True
