import numpy as np
import cv2
import time
import subprocess

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

    # # compute the width of the new image, which will be the
    # # maximum distance between bottom-right and bottom-left
    # # x-coordiates or the top-right and top-left x-coordinates
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))

    # compute the height of the new image, which will be the
    # maximum distance between the top-right and bottom-right
    # y-coordinates or the top-left and bottom-left y-coordinates
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))

    marker_px = min(maxWidth, maxHeight) # square markers so minimum of either dimension
    # not sure to use max or min here - i think max will require more interpolation since more of the markers will be farther away, aka less resolution
    # so then more computing will be required to make them bigger? not sure
    
    dst = np.array([
        [tl[0], tl[1]],  # the entire image is based on this dimension - aka this pixel dimension should be used to find real life dimension
        [tl[0] + marker_px, tl[1]],
        [tl[0] +  marker_px, tl[1] + marker_px],
        [tl[0], tl[1] + marker_px]], dtype = "float32")

    # compute the perspective transform matrix and then apply it
    M = cv2.getPerspectiveTransform(corners[0], dst) # use first marker to obtain transform
    warped = cv2.warpPerspective(image, M, (image.shape[0], image.shape[1]))

    # return the warped image
    return warped

def four_point_transform_multi(image, corners):     
    
    count = 0
    M = None
    for corner in corners:  
        # Get coordinate of each corner
        (tl, tr, br, bl) = corner[0]

        # # compute the width of the new image, which will be the
        # # maximum distance between bottom-right and bottom-left
        # # x-coordiates or the top-right and top-left x-coordinates
        widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
        widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
        maxWidth = max(int(widthA), int(widthB))

        # compute the height of the new image, which will be the
        # maximum distance between the top-right and bottom-right
        # y-coordinates or the top-left and bottom-left y-coordinates
        heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
        heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
        maxHeight = max(int(heightA), int(heightB))

        marker_px = min(maxWidth, maxHeight) # square markers so minimum of either dimension
        # not sure to use max or min here - i think max will require more interpolation since more of the markers will be farther away, aka less resolution
        # so then more computing will be required to make them bigger? not sure
        
        dst = np.array([
            [tl[0], tl[1]],  # the entire image is based on this dimension - aka this pixel dimension should be used to find real life dimension
            [tl[0] + marker_px, tl[1]],
            [tl[0] +  marker_px, tl[1] + marker_px],
            [tl[0], tl[1] + marker_px]], dtype = "float32")

        # compute the perspective transform matrix and then apply it
        M =+ cv2.getPerspectiveTransform(corners[0], dst)

        count += 1

    M = M/count # average of the matrices

    warped = cv2.warpPerspective(image, M, (image.shape[1], image.shape[0]))

    # return the warped image
    return warped


def marker_transform(frame, aruco_dict_type):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # more processing can be done to the images
    
    dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    corners, ids, rejected_img_points = detector.detectMarkers(gray)
    
    if len(corners) > 0:
        # return four_point_transform(frame, corners[0]) # USING ONLY SINGLE MARKER
        return four_point_transform_multi(frame, corners)
    else: 
        return frame

def stream(): 
    aruco_dict_type = ARUCO_DICT["DICT_6X6_100"]

    video = cv2.VideoCapture(0)

    time.sleep(2.0)

    while True:
        ret, frame = video.read()

        if not ret:
            break
        
        output = marker_transform(frame, aruco_dict_type)
    
        cv2.imshow('Output Result', output)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    video.release()
    cv2.destroyAllWindows()

if __name__ == "__main__": 
    stream()