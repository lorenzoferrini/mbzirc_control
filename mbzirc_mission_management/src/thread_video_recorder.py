from threading import Thread,Event
import cv2
from datetime import datetime
import os
#from Queue import Queue,Empty,Full
from matplotlib import image


class VideoWriterThreaded():
    def __init__(self):

        self.status = True
        now_string = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        dir_path = os.path.dirname(os.path.realpath(__file__))
        self.video_name = now_string + '.avi'
        self.image_directory = os.path.join(dir_path,now_string)
        try:
            os.mkdir(self.image_directory)
        except:
            raise
        self.frame =[]
        #self.queue = Queue(maxsize=5)
        self.start_recording_event = Event()
        self.recording_thread = Thread(target=self.recording_thread, args=())
        self.recording_thread.daemon = True
        self.recording_thread.start()

    def update_frame(self,frame):
        self.frame = frame


    def recording_thread(self):
        self.start_recording_event.wait()
        img_number_name = 0
        while self.status:
                try:
                    cv2.imwrite(os.path.join(self.image_directory, '%06d.jpg' % img_number_name),self.frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
		    #image.imsave(os.path.join(self.image_directory, '%06d.png' % img_number_name),self.frame)
                    img_number_name += 1
                except:
                    pass

    def stop(self):
        self.status = False

    def start_recording(self):
        self.start_recording_event.set()



