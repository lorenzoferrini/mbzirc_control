import faulthandler
faulthandler.enable()
import os.path

from threading import Thread
import cv2
from ConfigParser import SafeConfigParser

class VideoShow:
    """
    Class that continuously shows a frame using a dedicated thread.
    """

    def __init__(self, frame=None):
        self.frame = frame
        self.stopped = False
        path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "config/configuration.ini")
        self.config = SafeConfigParser()
        self.config.read(path)

    def start(self):
        Thread(target=self.show, args=()).start()
        return self

    def show(self):
        cv2.namedWindow('Video',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Video',self.config.getint('video_show','resize_width'),self.config.getint('video_show','resize_height'))
        while not self.stopped:
            cv2.imshow("Video", self.frame)
            cv2.waitKey(1)

    def stop(self):
        self.stopped = True
