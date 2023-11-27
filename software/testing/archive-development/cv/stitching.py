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
    for corner in corners: 
        accuracy = calc_accuracy(image, corner, 10)

        if accuracy > 0.7: 

            (tl, tr, br, bl) = corner[0]

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

            M = cv2.getPerspectiveTransform(corners[0], dst)
            warped = cv2.warpPerspective(image, M, (image.shape[0]+1000, image.shape[1]+1000))

            # Locate the marker being used to warp perspective
            frame = cv2.rectangle(warped, (int(tl[0]), int(tl[1])), (int(tl[0] - marker_px), int(tl[1] - marker_px)), (125, 125, 125), 2)

            # Cut out black edges 
            th = 1 # threshold for black edges
            y_nonzero, x_nonzero, _ = np.nonzero(frame>th)
            cropped = frame[np.min(y_nonzero):np.max(y_nonzero), np.min(x_nonzero):np.max(x_nonzero)]
            
            # while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
            #     cv2.imshow("Frame", cropped)

            return cropped

def marker_ids(frame, aruco_dict_type):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    corners, ids, rejected_img_points = detector.detectMarkers(gray)

    # Add Identification
    # aruco_display(corners, ids, rejected_img_points, frame)

    # while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
    #     cv2.imshow("Frame", frame)

    return corners, ids
    
def find_pose(corners, matrix_coefficients, distortion_coefficients):
    # Estimate pose of each marker and return the camera's rotational and translational vectors

    # Size of the marker in real life in mmm
    marker_size = 25 # mm
    
    # Object points
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
    
    if debug: 
        print("Error Percentage: {}%".format(percent))

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

def stitch_prepare(base, new, crop_factor, single_marker=False, debug=False): 
    aruco_dict_type = ARUCO_DICT["DICT_6X6_100"]

    # Collect IDs of all markers for each image
    corners_1, ids_1 = marker_ids(base, aruco_dict_type)
    corners_2, ids_2 = marker_ids(new, aruco_dict_type)

    markers_matrix = []
    tx_list = []
    ty_list = []
    tx = 0
    ty = 0 

    common = np.intersect1d(ids_1, ids_2)

    # Using the average of multiple markers to warp
    if not single_marker: 
        for item in common: 
            index_1 = np.where(ids_1 == item)[0][0]
            index_2 = np.where(ids_2 == item)[0][0]
            
            accuracy_1 = calc_accuracy(base, corners_1[index_1], 10)
            accuracy_2 = calc_accuracy(new, corners_2[index_2], 10)

            if accuracy_1 < 0.7 and accuracy_2 < 0.7: 
                h_matrix, _ = cv2.findHomography(corners_2[index_2], corners_1[index_1])
                
                markers_matrix.append(h_matrix)

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


        # Find the average of all the homography matrices
        h = np.mean(markers_matrix, axis=0)

        if debug: 
            print(h)

        # Find the average of translation for all markers
        tx = np.mean(tx_list, axis=0)
        ty = np.mean(ty_list, axis=0)


    # Using only a single marker to find warp and offset
    else: 
        # Find Marker with max accuracy
        i_max_accuracy_1 = 0
        i_max_accuracy_2 = 0
        last_accuracy_1 = 1

        for item in common:
            index_1 = np.where(ids_1 == item)[0][0]
            index_2 = np.where(ids_2 == item)[0][0]
            
            accuracy_1 = calc_accuracy(base, corners_1[index_1], 10)
            # accuracy_2 = calc_accuracy(new, corners_2[index_2], 10)

            if accuracy_1 < last_accuracy_1: 
                last_accuracy_1 = accuracy_1
                
                i_max_accuracy_1 = index_1
                i_max_accuracy_2 = index_2

        #     index_2 = np.where(ids_2 == item)[0][0]
        h, _ = cv2.findHomography(corners_2[i_max_accuracy_2], corners_1[i_max_accuracy_1])

        if corners_2[i_max_accuracy_2][0][0][0] > new.shape[0]/2: 
            tx = corners_2[i_max_accuracy_2][0][0][0] - corners_1[i_max_accuracy_1][0][0][0]
        else: 
            tx = corners_1[i_max_accuracy_1][0][0][0] - corners_2[i_max_accuracy_2][0][0][0]
        
        if corners_2[i_max_accuracy_2][0][0][1] > new.shape[1]/2: 
            ty = corners_2[i_max_accuracy_2][0][0][1] - corners_1[i_max_accuracy_1][0][0][1]
        else: 
            ty = corners_1[i_max_accuracy_1][0][0][1] - corners_2[i_max_accuracy_2][0][0][1]

    # Translation correction matrix
    translate = np.array([  [1,  0, -tx],
                            [0,  1, -ty], 
                            [0,  0,   1]], dtype="float64")
    
    # Apply translation
    M = np.matmul(h, translate)

    # Warp image
    result = cv2.warpPerspective(new, M, (base.shape[0]*3, base.shape[1]*3))
    
    # Crop out black edge
    # th = 1 # threshold for black edges
    # y_nonzero, x_nonzero, _ = np.nonzero(result>th)
    # cropped = result[np.min(y_nonzero):np.max(y_nonzero), np.min(x_nonzero):np.max(x_nonzero)]

    th = 1 # threshold for black edges
    y_nonzero, x_nonzero, _ = np.nonzero(result>th)
    cropped = result[np.min(y_nonzero)+crop_factor:np.max(y_nonzero)-crop_factor, np.min(x_nonzero)+crop_factor:np.max(x_nonzero)-crop_factor]

    if debug:
        while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
            cv2.imshow("Frame", cropped)

    return cropped

def flatten_final(image, crop_factor, single_marker=False, debug=False): 
    # Collect corners and IDs of all markers
    corners, ids = marker_ids(image, ARUCO_DICT["DICT_6X6_100"])

    # Using the average of multiple markers to warp
    if not single_marker: 
        markers_matrix = []
        tx_list = []
        ty_list = []
        tx = 0
        ty = 0 
    
        for corner in corners: 
            (tl, tr, br, bl) = corner[0]

            # compute the width of the new image, which will be the
            # maximum distance between bottom-right and bottom-left
            # x-coordinates or the top-right and top-left x-coordinates
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

            accuracy = calc_accuracy(image, corner, 10, debug=False)

            if accuracy < 5: 
                h_matrix, _ = cv2.findHomography(corner, dst)
                
                markers_matrix.append(h_matrix)

                if corner[0][0][0] > image.shape[0]/2: 
                    tx = corner[0][0][0] - dst[0][0]
                else: 
                    tx = dst[0][0] - corner[0][0][0]
                
                if corner[0][0][1] > image.shape[1]/2: 
                    ty = corner[0][0][1] - dst[0][1]
                else: 
                    ty = dst[0][1] - corner[0][0][1]

                tx_list.append(tx)
                ty_list.append(ty)

        if debug: 
            print(markers_matrix)
        # Find the average of all the homography matrices
        h = np.mean(markers_matrix, axis=0)

        # Find the average of translation for all markers
        tx = np.mean(tx_list, axis=0)
        ty = np.mean(ty_list, axis=0)

    # Using only a single marker to find warp and offset
    else: 
        # Find Marker with max accuracy
        i_max_accuracy = 0
        last_accuracy = 1
        tx = 0
        ty = 0 

        index = 0
        for corner in corners:
            accuracy = calc_accuracy(image, corner, 8)

            if accuracy < last_accuracy: 
                i_max_accuracy = index

            last_accuracy = accuracy
            index += 1

        h, _ = cv2.findHomography(corner, dst)

        if corner[i_max_accuracy][0][0] > image.shape[0]/2: 
            tx = corner[i_max_accuracy][0][0] - dst[0][0]
        else: 
            tx = dst[0][0] - corner[i_max_accuracy][0][0]
        
        if corner[i_max_accuracy][0][1] > image.shape[1]/2: 
            ty = corner[i_max_accuracy][0][1] - dst[0][1]
        else: 
            ty = dst[0][1] - corner[i_max_accuracy][0][1]

    # Translation correction matrix
    translate = np.array([  [1,  0, -tx],
                            [0,  1, -ty], 
                            [0,  0,   1]], dtype="float64")
    
    # Apply translation
    M = np.matmul(h, translate)

    # Warp image
    result = cv2.warpPerspective(image, M, (image.shape[0]*3, image.shape[1]*3))
    
    # Crop out black edge
    th = 1 # threshold for black edges
    y_nonzero, x_nonzero, _ = np.nonzero(result>th)
    cropped = result[np.min(y_nonzero)+crop_factor:np.max(y_nonzero)-crop_factor, np.min(x_nonzero)+crop_factor:np.max(x_nonzero)-crop_factor]

    if debug:
        while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
            cv2.imshow("Frame", cropped)

    return cropped

def blurry(image, threshold):
	# compute the Laplacian of the image and then return the focus
	# measure, which is simply the variance of the Laplacian
    blur = cv2.Laplacian(image, cv2.CV_64F).var()
    
    print(blur)
	
    if blur < threshold: 
        return True
    else: 
        return False

''' Main Program '''
def main(): 
    ''' UI to capture images per instructions '''
    # Directory of captured images
    dir = "./captured/"

    # Image Capturing Sequence
    cap = cv2.VideoCapture(0)
    cap.set(3, 1280) # set the resolution
    cap.set(4, 720)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)

    # time.sleep(1)

    # Index for loop
    i = 0

    time.sleep(1)

    while(True): 
        # Camera startup before taking a photo
        while not cap.isOpened(): # wait for camera
            pass
        else: 
            time.sleep(2)
            ret, frame = cap.read()

            if not ret:
                break

            # cv2.imshow('Captured Image', frame) # Display output 
            # cv2.waitKey(0)
            cv2.imwrite(dir + str(i) + ".jpg", frame)

            time.sleep(0.5)

        # First image
        if i == 0: 
            if blurry(frame, 200): 
                i = 0
            else: 
                i += 1
            
            # Back to top of loop to take next image
            continue

        # Use the first captured image as the base only if we are stitching the first two images
        if i == 1: # Captured two image
            base = cv2.imread(dir + "0.jpg") 

        # Other images
        else: 
            # Use the stitched base image
            base = cv2.imread("./result.jpg") 

        # Image to be added is the captured frame
        new = frame.copy()

        # Prepare image for stitching
        warped = stitch_prepare(base, new, crop_factor=50, single_marker=False, debug=True)
        
        # Save images for error checking
        cv2.imwrite(dir + "./result/base.jpg", base)
        cv2.imwrite(dir + "./result/new.jpg", warped)

        cv2.imshow("Confirm Your Images", np.concatenate([base, new]))
        cv2.waitKey(0)

        # Build absolute paths
        root_dir = os.path.dirname(os.path.abspath("."))
        print(root_dir + r'\marker_based_stitcher\raw\3\custom_stitcher.py')

        i += 1

        # Stitch them
        # p = subprocess.Popen(['py', r'C:\Users\Victor Zhang\Documents\GitHub\OS-Handheld-CNC\software\testing\CV\stitch\marker_based_stitcher\raw\3\custom_stitch.py', r'C:\Users\Victor Zhang\Documents\GitHub\OS-Handheld-CNC\software\testing\CV\stitch\marker_based_stitcher\raw\3\base.jpg', r'C:\Users\Victor Zhang\Documents\GitHub\OS-Handheld-CNC\software\testing\CV\stitch\marker_based_stitcher\raw\3\new.jpg', '--work_megapix', '0.6', '--features', 'orb', '--matcher', 'affine', '--estimator', 'affine', '--match_conf', '0.3', '--conf_thresh', '0.3', '--ba', 'affine', '--ba_refine_mask', 'xxxxx', '--wave_correct', 'no', '--warp', 'plane'])
        try: 
            p = subprocess.Popen(['py', r'C:\Users\victo\Documents\GitHub\OS-Handheld-CNC\software\testing\CV\stitch\marker_based_stitcher\raw\3\custom_stitch.py', r'C:\Users\victo\Documents\GitHub\OS-Handheld-CNC\software\testing\CV\stitch\marker_based_stitcher\raw\3\result\base.jpg', r'C:\Users\victo\Documents\GitHub\OS-Handheld-CNC\software\testing\CV\stitch\marker_based_stitcher\raw\3\result\new.jpg', '--work_megapix', '0.6', '--features', 'orb', '--matcher', 'homography', '--estimator', 'homography', '--match_conf', '0.3', '--conf_thresh', '0.5', '--ba', 'ray', '--ba_refine_mask', 'xxxxx', '--wave_correct', 'horiz', '--blend', 'multiband', '--warp', 'plane'])
            p.communicate()
        except: 
            print("Stitching failed (UI: retake photo)")
            break

    stitched = cv2.imread("result.jpg")
    
    try: 
        ''' Now we flatten the image '''
        flattened = flatten_final(stitched, 0, debug=True)

        while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
            cv2.imshow("Final", flattened)

        cv2.imwrite("final.jpg", flattened)
    except: 
        print("Something went wrong for end of stitching")

if __name__ == "__main__": 
    main()