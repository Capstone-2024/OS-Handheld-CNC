import cv2
import subprocess
import time

def variance_of_laplacian(image):
	# compute the Laplacian of the image and then return the focus
	# measure, which is simply the variance of the Laplacian
	return cv2.Laplacian(image, cv2.CV_64F).var()

array = [0, 50, 100, 255] # test different focus

# Test if focus can be changed using the v4l1 utils lib
i = 0 
for focus in array:
    cam_props = {'focus_auto': 0, 'focus_absolute': focus}

    for key in cam_props:
        subprocess.call(['v4l2-ctl -d /dev/video0 -c {}={}'.format(key, str(cam_props[key]))],
                        shell=True)
        
    test = cv2.VideoCapture(0, cv2.CAP_DSHOW)

    # Camera startup before taking a photo
    time.sleep(2.0)

    ret, frame = test.read()

    print(variance_of_laplacian(frame))

    # cv2.imwrite(frame, "./focus-test/focus" + i + ".png")

    # Increment
    i += 1