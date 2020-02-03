import cv2
import threading
import prctl
import cv2
import numpy as np
import math
import contours_filter_library
import memory_class
import traceback
import os.path

from VideoGet import VideoGet
from VideoShow import VideoShow
from streaming.upstream import Upstreamer

from thread_video_recorder import VideoWriterThreaded
from ConfigParser import SafeConfigParser

class CameraError(Exception):
   # Camera doesn't work properly
   pass

class RecognitionThread:
    def __init__(self, camera=0, video_show=False, video_streaming=False, video_rec=False):
        path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "config/configuration.ini")
        self.config = SafeConfigParser()
        self.config.read(path)
        self.running = True
        try:
            self.video_getter = VideoGet(self.config.getint('video_get','camera_address')).start()
        except CameraError:
            raise CameraError
        self.locked=0
        self.err_x_pix=0
        self.err_y_pix=0
        self.err_x_m=0
        self.err_y_m=0
        self.dist=0
        self.psi_err=0
        self.theta_err=0

        f = self.config.getfloat('recognition_thread','focal_length')  # focal length [m]
        w = self.config.getfloat('recognition_thread','sensor_width')  # sensor width [m]
        h = self.config.getfloat('recognition_thread','sensor_height')  # sensor height [m]

        self.resw = self.config.getfloat('video_get','width')  # dimensioni (larghezza) in pixel del frame da analizzare
        self.resh = self.config.getfloat('video_get','height')  # dimensioni (altezza) in pixel del frame da analizzare

        self.KNOWN_DISTANCE = self.config.getfloat('recognition_thread','known_distance')  # [m] measured
        self.KNOWN_RADIUS = self.config.getfloat('recognition_thread','known_radius')  # [m] measured
        self.KNOWN_W = w / f * self.KNOWN_DISTANCE  # Horizontal Field of View - calculated - if possible use measurement
        self.KNOWN_H = h / f * self.KNOWN_DISTANCE  # Vertical Field of View - calculated - if possible use measurement
        self.radius_cal = self.resw / self.KNOWN_W * self.KNOWN_RADIUS  # pixel according to resizedframe width - calculated without calibration through immagine_calibrazione.jpg

        self.lowerBoundList = [ self.config.getint('recognition_thread','lowerH'), self.config.getint('recognition_thread','lowerS'), self.config.getint('recognition_thread','lowerV')]
        self.upperBoundList = [self.config.getint('recognition_thread','upperH'), self.config.getint('recognition_thread','upperS'), self.config.getint('recognition_thread','upperV')]
        self.memory_frame = memory_class.Memory_Frame()
        self.reference_contour = contours_filter_library.reference_contour(self.config.get('recognition_thread','sagoma_path'))

        activation_flag=0
        if video_show:
            activation_flag = 1
        if video_streaming:
            activation_flag += 2
        if video_rec:
            activation_flag += 4

        if activation_flag == 0:
            self.start_recognize()
        elif activation_flag == 1:
            self.video_shower = VideoShow(self.video_getter.frame).start()
            self.start_showing()
        elif activation_flag == 2:
            self.up = Upstreamer("upstreaming thread", self.config.get('recognition_thread','server_address'), self.config.getint('recognition_thread','server_upstreaming_port'), False)
            self.up.start()
            self.start_recognize_and_stream()
        elif activation_flag == 3:
            self.video_shower = VideoShow(self.video_getter.frame).start()
            self.up = Upstreamer("upstreaming thread", self.config.get('recognition_thread','server_address'), self.config.getint('recognition_thread','server_upstreaming_port'), False)
            self.up.start()
            self.start_recognize_and_stream_and_show()
        elif activation_flag == 4:
            self.video_rec = VideoWriterThreaded()
            self.start_recognize_and_rec()
        elif activation_flag == 5:
            self.video_shower = VideoShow(self.video_getter.frame).start()
            self.video_rec = VideoWriterThreaded()
            self.start_showing_and_rec()
        elif activation_flag == 6:
            self.up = Upstreamer("upstreaming thread", self.config.get('recognition_thread','server_address'), self.config.getint('recognition_thread','server_upstreaming_port'), False)
            self.up.start()
            self.video_rec = VideoWriterThreaded()
            self.start_recognize_and_stream_and_rec()
        elif activation_flag == 7:
            self.video_shower = VideoShow(self.video_getter.frame).start()
            self.up = Upstreamer("upstreaming thread", self.config.get('recognition_thread','server_address'), self.config.getint('recognition_thread','server_upstreaming_port'), False)
            self.up.start()
            self.video_rec = VideoWriterThreaded()
            self.start_recognize_and_stream_and_show_and_rec()


    def calc_and_print_err(self, frame, x, y, radius, cframeX, cframeY, wframe, hframe):
        dist = self.KNOWN_DISTANCE * self.radius_cal / radius
        # disegna il cerchio che racchiude il pallone
        cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 1)
        err_x_pix = x - cframeX  # pixel
        err_y_pix = y - cframeY  # pixel

        W = self.KNOWN_W / self.KNOWN_DISTANCE * dist  # [m]
        H = self.KNOWN_H / self.KNOWN_DISTANCE * dist  # [m]
        risolW = W / wframe  # [m/pixel]
        risolH = H / hframe  # [m/pixel]
        err_x_m = risolW * err_x_pix  # [m]

        err_y_m = risolH * err_y_pix  # [m]
        psi_err = 180 / math.pi * math.atan(err_x_m / dist)  # [deg]
        theta_err = 180 / math.pi * math.atan(err_y_m / dist)  # [deg]
        cv2.putText(frame, "White_balloon", (int(x) - 100, int(y) - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        cv2.putText(frame, "dist=%.2fm" % (dist), (0, cframeY + 260), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, "radius=%.2fm" % (radius * risolH), (0, cframeY + 230), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (0, 255, 0), 2)
        cv2.putText(frame, "radius_pix=%.2fpix" % (radius), (0, cframeY + 200), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (0, 255, 0),
                    2)
        cv2.putText(frame, "psi=%.2fdeg" % (psi_err), (0, cframeY + 170), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(frame, "theta=%.2fdeg" % (theta_err), (0, cframeY + 140), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (0, 255, 0), 2)
        cv2.putText(frame, "err_y=%.2fm" % (err_y_m), (0, cframeY + 110), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(frame, "err_x=%.2fm" % (err_x_m), (0, cframeY + 80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        return err_x_pix, err_y_pix, err_x_m, err_y_m, dist, psi_err, theta_err

    # funzione di riconoscimento che restituisce le coordinate del centro inquadratura e, se rileva il pallone, restituisce le coordinate e il raggio del target
    def acquire_and_process_image(self, frame):

        #global KNOWN_DISTANCE, KNOWN_RADIUS, KNOWN_W, KNOWN_H, radius_cal, resw, resh

        kernelOpen = np.ones((5, 5))
        kernelClose = np.ones((49, 49))
        lowerBound = np.array(self.lowerBoundList)
        upperBound = np.array(self.upperBoundList)

        # flag aggancio bersaglio
        locked = 0

        hframe, wframe, channels = frame.shape
        # coordinate del centro inquadratura in pixel
        cframeX = int(wframe / 2)
        cframeY = int(hframe / 2)
        frame_center = (cframeX, cframeY)
        # disegna il centro inquadratura
        cv2.circle(frame, frame_center, 2, (0, 255, 0), 1)
        # filtro gaussiano
        blurred = cv2.GaussianBlur(frame, (7, 7), 0)
        # convert BGR to HSV
        frameHSV = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)
        # create the Mask
        mask = cv2.inRange(frameHSV, lowerBound, upperBound)
        # morphology
        maskOpen = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernelOpen)
        maskClose = cv2.morphologyEx(maskOpen, cv2.MORPH_CLOSE, kernelClose)

        maskFinal = maskClose

        contours, hieracy = cv2.findContours(maskFinal, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        cv2.drawContours(frame, contours, -1, (255, 0, 0), 2)

        contours = contours_filter_library.contours_area_shape_filter(frame, contours, 350, 0.25, 0.95)

        cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)

        if len(contours) != 0:

            contours_circles = contours_filter_library.contours_matchShape_nearest_balloon(contours, self.reference_contour)

            if contours_circles != []:

                target = contours_circles
                x_box, y_box, w_box, h_box = cv2.boundingRect(target)

                cv2.rectangle(frame, (x_box, y_box), (x_box + w_box, y_box + h_box), (0, 0, 255), 2)

                radius = max(w_box, h_box) / 2
                x = x_box + w_box / 2
                y = y_box + h_box / 2

                memory_ballon = self.memory_frame.ballon_insert(x, y, radius)

                if memory_ballon:
                    x = memory_ballon[0]
                    y = memory_ballon[1]
                    radius = memory_ballon[2]

                    err_x_pix, err_y_pix, err_x_m, err_y_m, dist, psi_err, theta_err = self.calc_and_print_err(frame, x, y,
                                                                                                          radius,
                                                                                                          cframeX,
                                                                                                          cframeY,
                                                                                                          wframe,
                                                                                                          hframe)

                    locked = 1
                    return frame, locked, err_x_pix, err_y_pix, err_x_m, err_y_m, dist, psi_err, theta_err
            else:
                memory_ballon = self.memory_frame.last_balloon()
                if memory_ballon:
                    err_x_pix, err_y_pix, err_x_m, err_y_m, dist, psi_err, theta_err = self.calc_and_print_err(frame,
                                                                                                          memory_ballon[
                                                                                                              0],
                                                                                                          memory_ballon[
                                                                                                              1],
                                                                                                          memory_ballon[
                                                                                                              2],
                                                                                                          cframeX,
                                                                                                          cframeY,
                                                                                                          wframe,
                                                                                                          hframe)

                    locked = 1
                    return frame, locked, err_x_pix, err_y_pix, err_x_m, err_y_m, dist, psi_err, theta_err
        else:
            memory_ballon = self.memory_frame.last_balloon()
            if memory_ballon:
                err_x_pix, err_y_pix, err_x_m, err_y_m, dist, psi_err, theta_err = self.calc_and_print_err(frame,
                                                                                                      memory_ballon[0],
                                                                                                      memory_ballon[1],
                                                                                                      memory_ballon[2],
                                                                                                      cframeX,
                                                                                                      cframeY, wframe,
                                                                                                      hframe)
                locked = 1
                return frame, locked, err_x_pix, err_y_pix, err_x_m, err_y_m, dist, psi_err, theta_err

        return frame, locked, [], [], [], [], [], [], []

    def ballonchecker(self):
        return self.locked,self.err_x_pix,self.err_y_pix, self.err_x_m, self.err_y_m, self.dist, self.psi_err, \
               self.theta_err

    def start_showing(self):
        prctl.set_name("RecognitionTread")
        # Create another thread to show/save frames
        def start_recognize_and_show_thread():
            prctl.set_name("Interno")
            while self.running:
                try:
                    frame = self.video_getter.frame
                    frame_ret, self.locked, self.err_x_pix, self.err_y_pix, self.err_x_m, self.err_y_m, self.dist,\
                    self.psi_err, self.theta_err= self.acquire_and_process_image(frame)
                    self.video_shower.frame = frame_ret
                except AttributeError:
                    pass
        self.recording_thread = threading.Thread(target=start_recognize_and_show_thread,name='recognitionandshowThread', args=())
        self.recording_thread.daemon = True
        self.recording_thread.start()

    def start_recognize(self):
        prctl.set_name("RecognizeTread")
        # Create another thread to show/save frames
        def start_recognize_thread():
            prctl.set_name("RecognizeTreadIN")
            while self.running:
                try:
                    frame = self.video_getter.frame
                    frame_ret, self.locked, self.err_x_pix, self.err_y_pix, self.err_x_m, self.err_y_m, self.dist,\
                    self.psi_err, self.theta_err= self.acquire_and_process_image(frame)
                except AttributeError:
                    pass
        self.recording_thread = threading.Thread(target=start_recognize_thread,name='recognitionThread', args=())
        self.recording_thread.daemon = True
        self.recording_thread.start()

    def start_recognize_and_stream(self):
        prctl.set_name("RecognizeTread")
        # Create another thread to show/save frames
        def start_recognize_and_stream_thread():
            prctl.set_name("Recognizestream")
            while self.running:
                try:
                    frame = self.video_getter.frame
                    frame_ret, self.locked, self.err_x_pix, self.err_y_pix, self.err_x_m, self.err_y_m, self.dist,\
                    self.psi_err, self.theta_err= self.acquire_and_process_image(frame)
                    self.up.stream_frame(cv2.resize(frame_ret,(self.config.getint('recognition_thread','resize_width_for_upstreaming'),self.config.getint('recognition_thread','resize_height_for_upstreaming'))))
                except AttributeError:
                    pass
        self.recording_thread = threading.Thread(target=start_recognize_and_stream_thread,name='recognitionThread', args=())
        self.recording_thread.daemon = True
        self.recording_thread.start()

    def start_recognize_and_stream_and_show(self):
        prctl.set_name("RecognizeTread")
        # Create another thread to show/save frames
        def start_recognize_and_stream_and_show_thread():
            prctl.set_name("RecognizestreamShow")
            while self.running:
                try:
                    frame = self.video_getter.frame
                    frame_ret, self.locked, self.err_x_pix, self.err_y_pix, self.err_x_m, self.err_y_m, self.dist,\
                    self.psi_err, self.theta_err = self.acquire_and_process_image(frame)
                    self.video_shower.frame = frame_ret
                    self.up.stream_frame(cv2.resize(frame_ret,(self.config.getint('recognition_thread','resize_width_for_upstreaming'),self.config.getint('recognition_thread','resize_height_for_upstreaming'))))
                except AttributeError:
                    pass
        self.recording_thread = threading.Thread(target=start_recognize_and_stream_and_show_thread,name='recognitionThread', args=())
        self.recording_thread.daemon = True
        self.recording_thread.start()

    def start_showing_and_rec(self):
        prctl.set_name("RecognitionTread")
        # Create another thread to show/save frames
        def start_recognize_and_show_thread():
            prctl.set_name("Interno")
            while self.running:
                try:
                    frame = self.video_getter.frame
                    frame_ret, self.locked, self.err_x_pix, self.err_y_pix, self.err_x_m, self.err_y_m, self.dist,\
                    self.psi_err, self.theta_err= self.acquire_and_process_image(frame)
                    self.video_shower.frame = frame_ret
                    self.video_rec.update_frame(frame_ret)
                except AttributeError:
                    pass
        self.recording_thread = threading.Thread(target=start_recognize_and_show_thread,name='recognitionandshowThread', args=())
        self.recording_thread.daemon = True
        self.recording_thread.start()

    def start_recognize_and_rec(self):
        prctl.set_name("RecognizeTread")
        # Create another thread to show/save frames
        def start_recognize_thread():
            prctl.set_name("RecognizeTreadIN")
            while self.running:
                try:
                    frame = self.video_getter.frame
                    frame_ret, self.locked, self.err_x_pix, self.err_y_pix, self.err_x_m, self.err_y_m, self.dist,\
                    self.psi_err, self.theta_err= self.acquire_and_process_image(frame)
                    self.video_rec.update_frame(frame_ret)
                except AttributeError:
                    pass
        self.recording_thread = threading.Thread(target=start_recognize_thread,name='recognitionThread', args=())
        self.recording_thread.daemon = True
        self.recording_thread.start()

    def start_recognize_and_stream_and_rec(self):
        prctl.set_name("RecognizeTread")
        # Create another thread to show/save frames
        def start_recognize_and_stream_thread():
            prctl.set_name("Recognizestream")
            while self.running:
                try:
                    frame = self.video_getter.frame
                    frame_ret, self.locked, self.err_x_pix, self.err_y_pix, self.err_x_m, self.err_y_m, self.dist,\
                    self.psi_err, self.theta_err= self.acquire_and_process_image(frame)
                    self.up.stream_frame(cv2.resize(frame_ret,(self.config.getint('recognition_thread','resize_width_for_upstreaming'),self.config.getint('recognition_thread','resize_height_for_upstreaming'))))
                    self.video_rec.update_frame(frame_ret)
                except AttributeError:
                    pass
        self.recording_thread = threading.Thread(target=start_recognize_and_stream_thread,name='recognitionThread', args=())
        self.recording_thread.daemon = True
        self.recording_thread.start()

    def start_recognize_and_stream_and_show_and_rec(self):
        prctl.set_name("RecognizeTread")
        # Create another thread to show/save frames
        def start_recognize_and_stream_and_show_thread():
            prctl.set_name("RecognizestreamShow")
            while self.running:
                try:
                    frame = self.video_getter.frame
                    frame_ret, self.locked, self.err_x_pix, self.err_y_pix, self.err_x_m, self.err_y_m, self.dist,\
                    self.psi_err, self.theta_err = self.acquire_and_process_image(frame)
                    self.video_shower.frame = frame_ret
                    self.up.stream_frame(cv2.resize(frame_ret,(self.config.getint('recognition_thread','resize_width_for_upstreaming'),self.config.getint('recognition_thread','resize_height_for_upstreaming'))))
                    self.video_rec.update_frame(frame_ret)
                except AttributeError:
                    pass
        self.recording_thread = threading.Thread(target=start_recognize_and_stream_and_show_thread,name='recognitionThread', args=())
        self.recording_thread.daemon = True
        self.recording_thread.start()


    def stop(self):
	self.video_getter.stop()
	try:
            self.video_shower.stop()
        except AttributeError:
            pass
	try:
	    self.up.close()
        except AttributeError:
            pass
        try:
	    self.video_rec.stop()
        except AttributeError:
            pass
        self.running = False
    
    def stop_recorder_and_start_new(self):
	try:
		old_video_rec = self.video_rec 
		self.video_rec = VideoWriterThreaded()
		old_video_rec.stop()
	except AttributeError:
            pass
    
    def start_recorder(self):
	try:
		self.video_rec.start_recording()
	except AttributeError:
            pass

