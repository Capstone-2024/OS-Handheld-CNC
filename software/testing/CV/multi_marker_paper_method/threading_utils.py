from threading import Thread
import cv2
from sys import platform
import subprocess

class WebcamVideoStream:
    def __init__(self, src=0):
        # initialize the video camera stream and read the first frame
        # from the stream
        self.stream = cv2.VideoCapture(src)

        # Camera Settings
        # LINUX
        if platform == "linux":
            cam_props = {'focus_auto': 1}

            for key in cam_props:
                subprocess.call(['v4l2-ctl -d /dev/video0 -c {}={}'.format(key, str(cam_props[key]))],
                            shell=True)

        else:
            # WINDOWS - does not work for the older version camera
            # video.set(cv2.CAP_PROP_FOCUS, 200)
            self.stream.set(cv2.CAP_PROP_AUTOFOCUS, 0) # auto focus
            focus = 35 # min: 0, max: 255, increment:5
            self.stream.set(cv2.CAP_PROP_FOCUS, focus)

        (self.grabbed, self.frame) = self.stream.read()
        # initialize the variable used to indicate if the thread should
        # be stopped
        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return
            # otherwise, read the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        # return the frame most recently read
        return self.frame

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True