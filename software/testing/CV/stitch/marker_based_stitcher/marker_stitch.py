'''
1. Find matching IDs in image 1 and 2
2. Get pose of all matching IDs - perform some operation to average the orientation (translation is different) of all? RANSAC or something?
3. Reproject BOTH images to the final pose
4. Choose one image to fix, probably the first image. 
5. Choose a matching marker, then apply an affine transformation on the second image. 
6. Overlay and blend the two images together (use stitcher?)
'''

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

    marker_px = min(maxWidth, maxHeight)
    
    dst = np.array([
        [tl[0], tl[1]],
        [tl[0] + marker_px, tl[1]],
        [tl[0] + marker_px, tl[1] + marker_px],
        [tl[0], tl[1] + marker_px]], dtype = "float32")

    # compute the perspective transform matrix and then apply it
    M = cv2.getPerspectiveTransform(corners[0], dst) # use first marker to obtain transform
    warped = cv2.warpPerspective(image, M, (image.shape[0]*2, image.shape[1]*2))

    #  frame = cv2.rectangle(warped, (tl[0], tl[1]), ((tl[0] +  marker_px), (tl[1] + marker_px)), (255, 0, 0), 1)

    frame = cv2.rectangle(warped, (int(tl[0]), int(tl[1])), (int(tl[0] +  marker_px), int(tl[1] + marker_px)), (125, 125, 125), 5)
    
    th = 1 # threshold for black edges
    y_nonzero, x_nonzero, _ = np.nonzero(frame>th)
    cropped = frame[np.min(y_nonzero):np.max(y_nonzero), np.min(x_nonzero):np.max(x_nonzero)]

    # frame = cv2.polylines(warped, [dst], True, (0,255,255))
    # return the warped and cropped image
    return cropped

def marker_ids(frame, aruco_dict_type):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    corners, ids, rejected_img_points = detector.detectMarkers(gray)

    return corners, ids
    
def find_pose(corners, matrix_coefficients, distortion_coefficients):
    # Estimate pose of each marker and return the camera's rotational and translational vectors
    
    # Still need a way to do this marker size, need to standardize the size 
    # Does not seem to actually affect the output values tho
    marker_size = 15
    
    # Object points
    objp = np.array([[-marker_size / 2, marker_size / 2, 0],
                            [marker_size / 2, marker_size / 2, 0],
                            [marker_size / 2, -marker_size / 2, 0],
                            [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)

    ret, rvec, tvec = cv2.solvePnP(objp, corners, matrix_coefficients, distortion_coefficients, False, cv2.SOLVEPNP_IPPE_SQUARE)
    
    return rvec, tvec

def main(): 
    aruco_dict_type = ARUCO_DICT["DICT_6X6_100"]

    # cap = cv2.VideoCapture(0)
    # time.sleep(1.0)
    # Capture 6 images, an image per 2 seconds
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
    
    
    # UI to capture images per instructions


    '''Iterate Through Images and Find Identical IDs'''
    ids_array = []
    unique_ids = [] # List of unique IDs
    corners_array = []

    files = os.listdir("./raw/2/")

    for file in files: 
        frame = cv2.imread("./raw/2/" + file)
        frame_cropped = frame[0:frame.shape[0], 0:frame.shape[1]-100] # Crop y by 100px - so that the image does not see the camera holder - change this for development

        # load numpy data files
        k = np.load("./calibration_matrix.npy")
        d = np.load("./distortion_coefficients.npy")

        # collect IDs of all markers for each image
        corners, ids = marker_ids(frame_cropped, aruco_dict_type, k, d)
        corners_array.append(corners)
        ids_array.append(ids)
        unique_ids.extend(ids)

        print(ids_array)

    unique_ids = np.unique(unique_ids) # filter for unique items

    print(unique_ids)
    
    id_pairs = {} # holds image index value of what images contain which marker
    # markerid1: [list of images], markerid2: [list of images], ...

    # Go from image 0 and then try to combine with image 1
    # rewrite transform function to tru



    for id in unique_ids: 
        in_list = [] # temp list of images index 

        i = 0
        for ids in ids_array: 
            if id in ids: 
                in_list.append(i)

            i += 1
        
        if in_list: 
            id_pairs[id] = in_list

    print(id_pairs)

    # Go through images and find 




if __name__ == "__main__": 
    main()