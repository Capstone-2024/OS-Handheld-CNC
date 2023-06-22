# Source: https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python/tree/main
# Command: py aruco_test.py --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy --type DICT_6X6_100

import numpy as np
import cv2
import sys
from utils import ARUCO_DICT, aruco_display
import argparse
import time


def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients, current_pose, prev_pose):

    '''
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera
    '''

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # more processing can be done 
    dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    # cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    # OLD NON-WORKING CODE
    # corners, ids, rejected_img_points = detector.detectMarkers(gray, dictionary,parameters=parameters,
    #     cameraMatrix=matrix_coefficients,
    #     distCoeff=distortion_coefficients)
    
    corners, ids, rejected_img_points = detector.detectMarkers(gray)

    # If markers are detected
    # most computing heavy
    if len(corners) > 0:

        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            # rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients, distortion_coefficients)
            # cv2.aruco.estimatePoseSingleMarkers deprecated, use solvePnP

            marker_size = 7
            
            # Object points
            objp = np.array([[-marker_size / 2, marker_size / 2, 0],
                                    [marker_size / 2, marker_size / 2, 0],
                                    [marker_size / 2, -marker_size / 2, 0],
                                    [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)

            ret, rvec, tvec = cv2.solvePnP(objp, corners[i], matrix_coefficients, distortion_coefficients, False, cv2.SOLVEPNP_IPPE_SQUARE)
            
            # XY rotation might not be useful since it will be fixed 
            # rotation around Z might be useful, since it can show us where the camera is pointing
        
            if str(ids[i]) in current_pose: 
                prev_pose[str(ids[i])] = current_pose[str(ids[i])]
                print("Marker {} moved by: X: {}, Y: {}, Z: {}".format(ids[i], [tvec - prev_pose[str(ids[i])]['translation']]))
        
            current_pose[str(ids[i])] = {'rotation': rvec,'translation': tvec}
            print(current_pose)


            # Draw a square around the markers
            # cv2.aruco.drawDetectedMarkers(frame, corners) 

            # Draw Axis
            #cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)
            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 4, 1)
            # print(rvec)
            # print(tvec)

    # put tag
    aruco_display(corners, ids, rejected_img_points, frame)

    return frame

if __name__ == '__main__':

    ap = argparse.ArgumentParser()
    ap.add_argument("-k", "--K_Matrix", required=True, help="Path to calibration matrix (numpy file)")
    ap.add_argument("-d", "--D_Coeff", required=True, help="Path to distortion coefficients (numpy file)")
    ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
    args = vars(ap.parse_args())

    
    if ARUCO_DICT.get(args["type"], None) is None:
        print(f"ArUCo tag type '{args['type']}' is not supported")
        sys.exit(0)

    aruco_dict_type = ARUCO_DICT[args["type"]]
    calibration_matrix_path = args["K_Matrix"]
    distortion_coefficients_path = args["D_Coeff"]
    
    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)

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
        
        # Record camera pose relative to each marker according to their unique id, using a dictionary
        current_pose = {}
        prev_pose = {}

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