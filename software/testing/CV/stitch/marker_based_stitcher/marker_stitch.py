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
    M = cv2.getPerspectiveTransform(corners[0], dst)
    warped = cv2.warpPerspective(image, M, (image.shape[0]*3, image.shape[1]*3))

    # Locate the marker being used to warp perspective
    frame = cv2.rectangle(warped, (int(tl[0]), int(tl[1])), (int(tl[0] +  marker_px), int(tl[1] + marker_px)), (125, 125, 125), 5)
    
    while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
        cv2.imshow("Frame", frame)

    th = 1 # threshold for black edges
    y_nonzero, x_nonzero, _ = np.nonzero(frame>th)
    
    cropped = frame[np.min(y_nonzero):np.max(y_nonzero), np.min(x_nonzero):np.max(x_nonzero)]
    
    while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
        cv2.imshow("Frame", cropped)


    ''' Attempt to have fully filled images '''

    # find all external contours in the threshold image then find
    # the *largest* contour which will be the contour/outline of
    # the stitched image
    gray = cv2.cvtColor(cropped.copy(), cv2.COLOR_BGR2GRAY)
    thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)[1]

    while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
        cv2.imshow("Frame", thresh)

    cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    c = max(cnts, key=cv2.contourArea)

    # allocate memory for the mask which will contain the
    # rectangular bounding box of the stitched image region
    mask = np.zeros(thresh.shape, dtype="uint8")
    (x, y, w, h) = cv2.boundingRect(c)
    cv2.rectangle(thresh, (x, y), (x + w, y + h), 255, -1)

    while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
        cv2.imshow("Frame", thresh)

    # frame = cv2.polylines(warped, [dst], True, (0,255,255))
    # return the warped and cropped image
    return cropped

def affine_transform(frame, src, dst): 
    # src is the image we want to transform, dst is the larger/full image
    M = cv2.getAffineTransform(src, dst)
    output = cv2.warpAffine(frame.copy(), M, frame.shape)

    while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
        cv2.imshow("Frame", output)
    
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
def calc_accuracy(img, corners, range=20):
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

    # while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
    #     cv2.imshow("Mask", stencil)

    sel = stencil != mask_value # select everything that is not mask_value
    image[sel] = fill_color # and fill it with fill_color

    # Replace pixels inside the marker with white as well
    cv2.fillPoly(image, np.int32([corners]), [255, 255, 255])

    # Show Image
    # while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
    #     cv2.imshow("Frame", image)

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
    # while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
    #     cv2.imshow("Frame", threshold)

    # print("Number of Pixels > Threshold: {}".format((threshold < 10).sum()))
    percent = (threshold <= 10).sum()/(threshold > 10).sum()*100

    # How much of the marker is showing as a percentage of the total space
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

    i = 0
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

        # print("Number of Markers: {}".format(len(ids)))

        # rvecs = [] # all rotational vectors
        # tvecs = [] # all translational vectors

        j = 0
        most_accurate = 0 # start at 100% error
        last_error_rate = 1
        accuracy_test = True # Enable accuracy test or not

        for corner in corners: # For each marker detected

            # If accuracy test is enabled
            if accuracy_test: 

                # Change offset range depending on marker size
                error_rate = calc_accuracy(frame_cropped, corner, 25) 
                
                # Only consider the marker if it has less than 0.7% of black pixels (indication of accuracy)
                if error_rate < 0.7: 
                    if error_rate < last_error_rate: 
                        most_accurate = ids[j]

            j += 1
        
        print(most_accurate)

        # Now transform the image using the corners of the most accurate marker
        corner_index = np.where(ids == most_accurate)[0]
        print(corner_index)

        output = four_point_transform(frame, corners[corner_index[0]])

        cv2.imwrite("./processed/" + str(i) + ".jpg", output)


        i += 1
    


    ''' Now look at the processed files '''
    files_new = os.listdir("./raw/2/")
    for file in files_new: 
        # Process image
        frame = cv2.imread("./raw/2/" + file)

        # source comes from image 2, destination comes from image 1 
        # affine_transform()
        
        # Collect IDs of all markers for each image
        corners, ids = marker_ids(frame, aruco_dict_type)
        

        # for corner in corners: # For each marker detected

        #     # Only add the data if they pass the accuracy test
        #     if accuracy_test: 

        #         # Change offset range depending on marker size
        #         error_rate = calc_accuracy(frame_cropped, corner, 25) 
                
        #         if error_rate < 0.7: 
        #             if error_rate < last_error_rate: 
        #                 most_accurate = ids[j]

                    # rvec, tvec = find_pose(corner, k, d)
                    # print("{}: {}".format(ids[j], rvec))

                    # rvecs.append(rvec)
                    # tvecs.append(tvec)
            
            # Run anyways if not using accuracy test
            # else: 
                # rvec, tvec = find_pose(corner, k, d)
                # print("{}: {}".format(ids[j], rvec))

                # rvecs.append(rvec)
                # tvecs.append(tvec)
  
            # j += 1 
        
        # Find Average
        # r = np.array(rvecs)
        # t = np.array(tvecs)

        # print("Rotation Avg: {}".format(r.mean(axis=0))) # MEAN DES NOT WORK :(, we need to do something else about this. 
        # Currently considering finding the perspective matrix or something. 
        # Or maybe just choose a marker that isn't warped and then combine idk

        # print("Translational Avg: {}".format(t.mean(axis=0)))
        
        # corners_array.append(corners)
        # ids_array.append(ids)
        # unique_ids.extend(ids)

        # print(ids_array)

    
    # unique_ids = np.unique(unique_ids) # filter for unique items

    # print(unique_ids)
    
    # id_pairs = {} # holds image index value of what images contain which marker
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