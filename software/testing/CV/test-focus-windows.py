import cv2
import subprocess
import time
import numpy as np

def variance_of_laplacian(image):
	# compute the Laplacian of the image and then return the focus
	# measure, which is simply the variance of the Laplacian
	return cv2.Laplacian(image, cv2.CV_64F).var()



array = np.arange(90, 255, 5).tolist()

i = 0 
for focus in array:

    cap = cv2.VideoCapture(0)

    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # auto focus
    cap.set(cv2.CAP_PROP_FOCUS, focus)

    time.sleep(1.0)

    # Camera startup before taking a photo
    while not cap.isOpened(): # wait for camera
        pass
    else: 
        ret, frame = cap.read()

        filename = "./focus-test/focus" + str(i) + ".png"
        
        cv2.imwrite(filename, frame)

        blur = variance_of_laplacian(frame) # higher is less blurry
        print(f'Focus: {focus}: {round(blur, 2)}')

        cap.release()

        time.sleep(1.0)

        # Increment
        i += 1