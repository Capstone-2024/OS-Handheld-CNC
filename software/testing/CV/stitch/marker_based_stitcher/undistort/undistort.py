import cv2
import os
import numpy as np

dir = "."
files = os.listdir(dir)

d = np.load("distortion_coefficients_new.npy")
k = np.load("calibration_matrix_new.npy")

def undistort_image(k, d, image): 
    h, w = image.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(k, d, (w,h), 1, (w,h))
    dst = cv2.undistort(image, k, d, None, newcameramtx)
    # Crop the image
    # x, y, w, h = roi
    # undistorted = dst[y:y+h, x:x+w]

    return dst

def main():
    for i in range(0, len(files)): 
        if files[i].endswith(".jpg"):
            image = cv2.imread(files[i])
            image = undistort_image(k, d, image)
            cv2.imshow('Image', image)
            cv2.waitKey(0)

    
if __name__ == "__main__": 
    main()