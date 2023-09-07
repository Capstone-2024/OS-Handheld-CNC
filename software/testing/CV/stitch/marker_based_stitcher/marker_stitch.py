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

    aruco_display(corners, ids, rejected_img_points, frame)

    # while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
    #     cv2.imshow("Frame", frame)

    return corners, ids
    
def find_pose(corners, matrix_coefficients, distortion_coefficients):
    # Estimate pose of each marker and return the camera's rotational and translational vectors

    # Size of the marker in real life in mmm
    marker_size = 15 # mm
    
    # Object points
    # objp = np.zeros((6*7,3), np.float32)
    # objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)  
    objp = np.array([[-marker_size / 2, marker_size / 2, 0],
                            [marker_size / 2, marker_size / 2, 0],
                            [marker_size / 2, -marker_size / 2, 0],
                            [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)

    ret, rvec, tvec = cv2.solvePnP(objp, corners, matrix_coefficients, distortion_coefficients, False, cv2.SOLVEPNP_IPPE_SQUARE)
    
    return rvec, tvec

''' Offset marker corners and compare black spots to white (or other colors) '''
def calc_accuracy(frame, corners, range=10): 
    (tl, tr, br, bl) = corners[0]
    tl[0] -= range # x to the left
    tr[1] -= range # y up

    tr[0] += range # x to the right
    tr[1] -= range # y up

    bl[0] -= range # x to the left
    br[1] += range # y down

    br[0] += range # x to the right
    br[1] += range # y down

    offset_contour = np.stack([tl, tr, br, bl])
    print(offset_contour)
    # fill anything outside of this box with white pixels 
    fill_color = [255, 255, 255] # any BGR color value to fill with
    mask_value = 255            # 1 channel white (can be any non-zero uint8 value)

    stencil  = np.zeros(frame.shape[:-1]).astype(np.uint8)
    cv2.fillPoly(stencil, offset_contour, mask_value)

    sel = stencil != mask_value # select everything that is not mask_value
    frame[sel] = fill_color # and fill it with fill_color

    # Replace pixels inside the marker with white as well
    cv2.fillPoly(frame, corners, mask_value)

    # Show Image
    while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
        cv2.imshow("Frame", frame)

    # Find max and mins of the offset box and crop
    x_max = max(tl[0], tr[0], bl[0], br[0])
    y_max = max(tl[1], tr[1], bl[1], br[1])

    x_min = min(tl[0], tr[0], bl[0], br[0])
    y_min = min(tl[1], tr[1], bl[1], br[1])

    frame_cropped = frame[x_min: x_max, y_min: y_max]

    # Show Image
    while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
        cv2.imshow("Frame", frame_cropped)



def aruco_display(corners, ids, rejected, image, terminal_print=False):
	if len(corners) > 0:
		# flatten the ArUco IDs list
		ids = ids.flatten()
		# loop over the detected ArUCo corners
		for (markerCorner, markerID) in zip(corners, ids):
			# extract the marker corners (which are always returned in
			# top-left, top-right, bottom-right, and bottom-left order)
			corners = markerCorner.reshape((4, 2))
			(topLeft, topRight, bottomRight, bottomLeft) = corners
			# convert each of the (x, y)-coordinate pairs to integers
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))

			cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
			cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
			cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
			cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
			# compute and draw the center (x, y)-coordinates of the ArUco
			# marker
			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
			# draw the ArUco marker ID on the image
			cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 255, 0), 2)
			if terminal_print: 
				print("[Inference] ArUco marker ID: {}".format(markerID))
			# show the output image
	return image

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
        print(file + ":")

        # Process image
        frame = cv2.imread("./raw/2/" + file)
        frame_cropped = frame[0:frame.shape[0], 0:frame.shape[1]-80] # Crop y by 100px - so that the image does not see the camera holder - change this for development

        # Load numpy data files
        k = np.load("./calibration_matrix.npy")
        d = np.load("./distortion_coefficients.npy")

        # Collect IDs of all markers for each image
        corners, ids = marker_ids(frame_cropped, aruco_dict_type)

        print("Number of Markers: {}".format(len(ids)))
        
        # print(corners)

        j = 0

        rvecs = [] # all rotational vectors
        tvecs = [] # all translational vectors
        for corner in corners: # For each marker detected
            print(corner)
            calc_accuracy(frame_cropped, corner)

            rvec, tvec = find_pose(corner, k, d)

            rvecs.append(rvec)
            tvecs.append(tvec)

            # print("{}: {}, {}".format(ids[j], rvec, tvec))

            j += 1 
        
        # Find Average
        r = np.array(rvecs)
        # t = np.array(tvecs)

        print("Rotation Avg: {}".format(r.mean(axis=0)))
        # print("Translational Avg: {}".format(t.mean(axis=0)))
        
        # corners_array.append(corners)
        # ids_array.append(ids)
        # unique_ids.extend(ids)

        # print(ids_array)

    
    # unique_ids = np.unique(unique_ids) # filter for unique items

    # print(unique_ids)
    
    id_pairs = {} # holds image index value of what images contain which marker
    # markerid1: [list of images], markerid2: [list of images], ...

    # Go from image 0 and then try to combine with image 1
    # rewrite transform function to tru


    # for id in unique_ids: 
    #     in_list = [] # temp list of images index 

    #     i = 0
    #     for ids in ids_array: 
    #         if id in ids: 
    #             in_list.append(i)

    #         i += 1
        
    #     if in_list: 
    #         id_pairs[id] = in_list

    # print(id_pairs)

    # Go through images and find 




if __name__ == "__main__": 
    main()