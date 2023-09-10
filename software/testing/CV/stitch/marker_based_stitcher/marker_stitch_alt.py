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
import imutils
import math

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
        [tl[0] - marker_px, tl[1]],
        [tl[0] - marker_px, tl[1] - marker_px],
        [tl[0], tl[1] - marker_px]], dtype = "float32")

    # compute the perspective transform matrix and then apply it
    M = cv2.getPerspectiveTransform(corners[0], dst)
    warped = cv2.warpPerspective(image, M, (image.shape[0]*4, image.shape[1]*4))

    # Locate the marker being used to warp perspective
    frame = cv2.rectangle(warped.copy(), (int(tl[0]), int(tl[1])), (int(tl[0] -  marker_px), int(tl[1] - marker_px)), (125, 125, 125), 5)
    
    # while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
    #     cv2.imshow("Frame", frame)

    # Cut out black edges 
    th = 1 # threshold for black edges
    y_nonzero, x_nonzero, _ = np.nonzero(warped>th)
    
    cropped = warped[np.min(y_nonzero):np.max(y_nonzero), np.min(x_nonzero):np.max(x_nonzero)]
    
    # while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
    #     cv2.imshow("Frame", cropped)

    ''' Attempt to have fully filled images '''

    # find all external contours in the threshold image then find
    # the *largest* contour which will be the contour/outline of
    # the stitched image
    gray = cv2.cvtColor(cropped.copy(), cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)[1]

    # while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
    #     cv2.imshow("Frame", thresh)

    cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    c = max(cnts, key=cv2.contourArea)

    # allocate memory for the mask which will contain the
    # rectangular bounding box of the stitched image region
    mask = np.zeros(thresh.shape, dtype="uint8")
    (x, y, w, h) = cv2.boundingRect(c)
    cv2.rectangle(thresh, (x, y), (x + w, y + h), 255, -1)

    # while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
    #     cv2.imshow("Frame", thresh)

    # frame = cv2.polylines(warped, [dst], True, (0,255,255))
    # return the warped and cropped image
    return cropped

def affine_transform(frame, src, dst): 
    # src is the image we want to transform, dst is the larger/full image
    M = cv2.getAffineTransform(src, dst)
    output = cv2.warpAffine(frame.copy(), M, frame.shape)

    # while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
    #     cv2.imshow("Frame", output)
    
    return output

def marker_ids(frame, aruco_dict_type):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    corners, ids, rejected_img_points = detector.detectMarkers(gray)

    # aruco_display(corners, ids, rejected_img_points, frame)

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
def calc_accuracy(img, corners, range=20, debug=False):
    image = img.copy()

    (tl, tr, br, bl) = corners[0].copy()

    # if offset range is larger than the corner pixel location then it cannot be subtracted, then just use the minimum value which is 0
    # or if addition results in image larger than the captured image, then use the max dimension value
    
    # Top Left
    if tl[0]+range <= image.shape[1]: 
        tl[0] += range # x to the left
    else: 
        tl[0] = image.shape[0]
    if tl[1]+range <= image.shape[0]: 
        tl[1] += range # y up
    else: 
        tl[1] = image.shape[1]

    # Top Right
    if tr[0] >= range: 
        tr[0] -= range # x to the right
    else:
        tr[0] = 0
    if tr[1]+range <= image.shape[0]: 
        tr[1] += range # y up
    else: 
        tr[1] = image.shape[0]

    # Bottom Left
    if bl[0]+range <= image.shape[1]: 
        bl[0] += range # x to the left
    else:
        bl[0] = image.shape[1]
    if bl[1] >= range: 
        bl[1] -= range # y down
    else: 
        bl[1] = 0

    # Bottom Right
    if br[0] >= range: 
        br[0] -= range # x to the right
    else: 
        br[0] = 0
    if br[1] >= range: 
        br[1] -= range # y down
    else: 
        br[1] = 0

    offset_contour = np.stack([tl, tr, br, bl])

    # fill anything outside of this box with white pixels 
    fill_color = [255, 255, 255] # any BGR color value to fill with
    mask_value = 255 # 1 channel white (can be any non-zero uint8 value)

    stencil = np.zeros(image.shape[:-1]).astype(np.uint8)
    cv2.fillPoly(stencil, np.int32([offset_contour]), mask_value)

    # Show Image
    if debug: 
        while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
            cv2.imshow("Mask: press 'q' to quit", stencil)

    sel = stencil != mask_value # select everything that is not mask_value
    image[sel] = fill_color # and fill it with fill_color

    # Replace pixels inside the marker with white as well
    cv2.fillPoly(image, np.int32([corners]), [255, 255, 255])

    # Show Image
    if debug: 
        while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
            cv2.imshow("Frame: press 'q' to quit", image)

    # Find max and mins of the offset box and crop
    y_max = int(max(tl[0], tr[0], bl[0], br[0]))
    x_max = int(max(tl[1], tr[1], bl[1], br[1]))

    y_min = int(min(tl[0], tr[0], bl[0], br[0]))
    x_min = int(min(tl[1], tr[1], bl[1], br[1]))

    # print(x_max, x_min, y_max, y_min)

    cropped = image[x_min: x_max, y_min: y_max]
    gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (11,11), 0)
    threshold = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 3, 2)

    # Show Image
    if debug: 
        while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
            cv2.imshow("Frame: press 'q' to quit", threshold)

    # How much of the marker is showing as a percentage of the total space
    percent = (threshold <= 10).sum()/(threshold > 10).sum()*100
    # print("Error Percentage: {}%".format(percent))

    return percent

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

def stitch_prepare(base, new, debug=False): 
    aruco_dict_type = ARUCO_DICT["DICT_6X6_100"]

    ''' Now look at the processed files '''
    # base = cv2.imread("./processed_alt/base.jpg") # first frame to continually add to
    # new = cv2.imread("./raw/3/2.jpg") # image to be transformed and added to base

    # Collect IDs of all markers for each image
    corners_1, ids_1 = marker_ids(base, aruco_dict_type)
    corners_2, ids_2 = marker_ids(new, aruco_dict_type)

    markers_matrix = []
    tx_list = []
    ty_list = []

    common = np.intersect1d(ids_1, ids_2)

    for item in common: 
        index_1 = np.where(ids_1 == item)[0][0]
        index_2 = np.where(ids_2 == item)[0][0]
        
        accuracy_1 = calc_accuracy(base, corners_1[index_1], 10)
        accuracy_2 = calc_accuracy(new, corners_2[index_2], 10)

        if accuracy_1 < 0.7 and accuracy_2 < 0.7: 
            h, _ = cv2.findHomography(corners_2[index_2], corners_1[index_1])
            # print(h)
            markers_matrix.append(h)

            if corners_2[index_2][0][0][0] > new.shape[0]/2: 
                x_offset = corners_2[index_2][0][0][0] - corners_1[index_1][0][0][0]
            else: 
                x_offset = corners_1[index_1][0][0][0] - corners_2[index_2][0][0][0]
            
            if corners_2[index_2][0][0][1] > new.shape[1]/2: 
                y_offset = corners_2[index_2][0][0][1] - corners_1[index_1][0][0][1]
            else: 
                y_offset = corners_1[index_1][0][0][1] - corners_2[index_2][0][0][1]

            tx_list.append(x_offset)
            ty_list.append(y_offset)

    # # Find the average of all the homography matrices
    avg_h = np.mean(markers_matrix, axis=0)

    # # Find the average of translation for all markers
    tx = np.mean(tx_list, axis=0)
    ty = np.mean(ty_list, axis=0)

    # Apply translation correction
    translate = np.array([[1, 0, -tx],
              [0, 1, -ty], 
              [0, 0, 1]])

    M = np.matmul(avg_h, translate)

    result = cv2.warpPerspective(new, M, (base.shape[0]*3, base.shape[1]*3))
    
    # Crop out black edge
    th = 1 # threshold for black edges
    y_nonzero, x_nonzero, _ = np.nonzero(result>th)
    cropped = result[np.min(y_nonzero):np.max(y_nonzero), np.min(x_nonzero):np.max(x_nonzero)]

    if debug:
        while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
            cv2.imshow("Frame", cropped)

    return cropped

    # cv2.imwrite("./processed_alt/test.jpg", cropped)

def main(): 
    ''' UI to capture images per instructions '''
    files = os.listdir("./raw/3/")

    for i in range(0, len(files)-1): 
        # Prepare Images
        base = cv2.imread(file[i])
        new = cv2.imread(file[i+1])

        stitch_prepare(base, new, True)

        # Stitch them


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

if __name__ == "__main__": 
    main()