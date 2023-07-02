import numpy as np
import cv2
import sys
from utils import ARUCO_DICT, aruco_display
import argparse
import time


def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients, current_pose, prev_pose):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # more processing can be done to the images
    
    dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    corners, ids, rejected_img_points = detector.detectMarkers(gray)

    # Marker Detection and Pose Estimation
    # Most computing heavy
    if len(corners) > 0:

        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the camera's rotational and translational vectors
            
            # Still need a way to do this marker size, need to standardize the size 
            # Does not seem to actually affect the output values tho
            marker_size = 7
            
            # Object points
            objp = np.array([[-marker_size / 2, marker_size / 2, 0],
                                    [marker_size / 2, marker_size / 2, 0],
                                    [marker_size / 2, -marker_size / 2, 0],
                                    [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)

            ret, rvec, tvec = cv2.solvePnP(objp, corners[i], matrix_coefficients, distortion_coefficients, False, cv2.SOLVEPNP_IPPE_SQUARE)
            
            # XY rotation might not be useful since it will be fixed 
            # rotation around Z might be useful, since it can show us where the camera is pointing
        
            if str(ids[i][0]) in current_pose: 
                prev_pose[str(ids[i][0])] = current_pose[str(ids[i][0])]
                difference = tvec - prev_pose[str(ids[i][0])]['translation']
                print("Marker {} moved by: X: {}, Y: {}, Z: {}".format(ids[i][0], difference[0], difference[1], difference[2]))
        
            current_pose[str(ids[i][0])] = {'rotation': rvec,'translation': tvec}
            # print(current_pose)

            # Draw Axis
            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 4, 1)

    # Put tag
    aruco_display(corners, ids, rejected_img_points, frame)

    return frame

def stream(): 
    aruco_dict_type = ARUCO_DICT["DICT_6X6_100"]

    # Record camera pose relative to each marker according to their unique id, using a dictionary
    current_pose = {}
    prev_pose = {}
    
    # load numpy data files
    k = np.load("calibration_matrix.npy")
    d = np.load("distortion_coefficients.npy")

    video = cv2.VideoCapture(0)

    # used to record the time when we processed last frame
    prev_frame_time = 0
    
    # used to record the time at which we processed current frame
    new_frame_time = 0

    time.sleep(2.0)

    while True:
        ret, frame = video.read()

        if not ret:
            break
        
        output = pose_estimation(frame, aruco_dict_type, k, d, current_pose, prev_pose)
        
        # FPS
        new_frame_time = time.time()
        fps = 1/(new_frame_time-prev_frame_time)
        prev_frame_time = new_frame_time
        fps = int(fps)
        fps = str(fps)
        font = cv2.FONT_HERSHEY_PLAIN
        cv2.putText(frame, fps, (7, 70), font, 1, (100, 255, 0), 3, cv2.LINE_AA)

        cv2.imshow('Output Result', output)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    video.release()
    cv2.destroyAllWindows()