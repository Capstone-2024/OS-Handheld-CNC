# Generate Images for Stitcher

import cv2
import time
import numpy as np
import subprocess
import os

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

def four_point_transform(image, corners):      
    # Get coordinate of each corner
    (tl, tr, br, bl) = corners[0]

    # compute the width of the new image, which will be the
    # maximum distance between bottom-right and bottom-left
    # x-coordiates or the top-right and top-left x-coordinates
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))

    # compute the height of the new image, which will be the
    # maximum distance between the top-right and bottom-right
    # y-coordinates or the top-left and bottom-left y-coordinates
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))

    marker_px = max(maxWidth, maxHeight)
    
    dst = np.array([
        [tl[0], tl[1]],
        [tl[0] + marker_px, tl[1]],
        [tl[0] +  marker_px, tl[1] + marker_px],
        [tl[0], tl[1] + marker_px]], dtype = "float32")

    # compute the perspective transform matrix and then apply it
    M = cv2.getPerspectiveTransform(corners[0], dst) # use first marker to obtain transform
    warped = cv2.warpPerspective(image, M, (image.shape[0], image.shape[1]))

    # return the warped image
    return warped

def marker_transform(frame, aruco_dict_type):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # more processing can be done to the images

    dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    corners, ids, rejected_img_points = detector.detectMarkers(gray)
    
    if len(corners) > 0:
        return four_point_transform(frame, corners[0])
    else: 
        return frame


def main(): 
    aruco_dict_type = ARUCO_DICT["DICT_6X6_100"]

    # cap = cv2.VideoCapture(0)
    # time.sleep(1.0)
    
    # for i in range(1, 7): 
    #     time.sleep(0.5)
    #     ret, frame = cap.read()

    #     if not ret:
    #         break
    #     cv2.imshow('Captured Image', frame) # Display output 
    
    #     frame_cropped = frame[0:frame.shape[0], 0:frame.shape[1]-100] # Crop y by 100px

    #     output = marker_transform(frame_cropped, aruco_dict_type)

    #     cv2.imwrite("./captured/" + str(i) + ".jpg", output)
    #     cv2.imshow('Captured Image', output) # Display output 
        
    #     time.sleep(2)
    #     cv2.destroyAllWindows()
    
    files = os.listdir("./raw/")
    print(files)

    i = 0 
    for file in files: 
        frame = cv2.imread("./raw/" + file)
        frame_cropped = frame[0:frame.shape[0], 0:frame.shape[1]-100] # Crop y by 100px

        output = marker_transform(frame_cropped, aruco_dict_type)

        gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY) # more processing can be done to the images

        ret, image = cv2.threshold(gray, 170, 100, cv2.THRESH_OTSU)

        # image = cv2.adaptiveThreshold(gray, 130, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)

        cv2.imwrite("./captured/" + str(i) + ".jpg", image)

        i += 1

if __name__ == "__main__": 
    main()
