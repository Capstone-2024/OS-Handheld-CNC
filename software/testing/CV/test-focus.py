import cv2
import subprocess
import time

def variance_of_laplacian(image):
	# compute the Laplacian of the image and then return the focus
	# measure, which is simply the variance of the Laplacian
	return cv2.Laplacian(image, cv2.CV_64F).var()

array = [5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19] # test different focus

# Test if focus can be changed using the v4l1 utils lib
i = 0 
for focus in array:
    cam_props = {'focus_absolute': focus, 'focus_auto': 0}

    for key in cam_props:
        subprocess.call(['v4l2-ctl -d /dev/video0 -c {}={}'.format(key, str(cam_props[key]))],
                        shell=True)

    cap = cv2.VideoCapture(0)
    # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

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