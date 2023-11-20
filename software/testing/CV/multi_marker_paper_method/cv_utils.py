from threading_utils import ARUCO_DICT, aruco_display
import cv2
import numpy as np
import time
import pandas as pd
import matplotlib.pyplot as plt
import math
from sys import platform
import subprocess

def pose_estimation(frame, marker_locations):
    # Pose Estimation Initialize 
    aruco_dict_type = ARUCO_DICT["DICT_6X6_250"]
    # matrix_coefficients = np.load("./calibration_matrix.npy")
    # distortion_coefficients = np.load("./distortion_coefficients.npy")

    # New Camera
    matrix_coefficients = np.load("./calibration_matrix_2.npy")
    distortion_coefficients = np.load("./distortion_coefficients_2.npy")

    # more processing can be done to the images
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Marker Detection
    dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    corners, ids, rejected_img_points = detector.detectMarkers(gray)

    ''' Pose Estimation (Most computing heavy) '''
    if len(corners) > 0:

        # Get an array of all available markers
        available_ids = marker_locations.keys()
        
        # Reference Marker Vars
        ref_id = ids[0]
        min_d = float('inf') # arbitrarily large distance

        rot_matrices = []
        tvecs = []

        marker_distortions = []
        total_distortion = 0 
        areas = []
        total_area = 0 

        # For each detected ID
        for i in range(0, len(ids)):

            # Check if the marker is in the stitched images
            if ids[i][0] in available_ids:

                # Size of the marker in real life in mmm
                marker_size = 25.41  # mm

                # Object points
                objp = np.array([[-marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, -marker_size / 2, 0],
                                [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
                
                mtx, roi = cv2.getOptimalNewCameraMatrix(matrix_coefficients, distortion_coefficients, (frame.shape[0], frame.shape[1]), 0.5, (frame.shape[0], frame.shape[1]))
                ret, rvec, tvec = cv2.solvePnP(objp, corners[i], mtx, None, False)

                # Add Rotation Matrix 
                rot_matrices.append(cv2.Rodrigues(rvec)[0])
                tvecs.append(tvec)

                # Find Marker Center
                (topLeft, topRight, bottomRight, bottomLeft) = corners[i][0]
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)

                # Image Center
                image_cX = int(frame.shape[0]/2.0)
                image_cY = int(frame.shape[1]/2.0)

                # Distance from Marker to Image Center
                d = math.dist([image_cX, image_cY], [cX, cY])

                # Calculate distances from center of each corner
                d_i_1 = math.dist([topLeft[0], topLeft[1]], [cX, cY])
                d_i_2 = math.dist([topRight[0], topRight[1]], [cX, cY])
                d_i_3 = math.dist([bottomRight[0], bottomRight[1]], [cX, cY])
                d_i_4 = math.dist([bottomLeft[0], bottomLeft[1]], [cX, cY])
                d_avg = np.average([d_i_1, d_i_2, d_i_3, d_i_4])
                v_i = 1/4*((d_i_1-d_avg)**2 + (d_i_2-d_avg)**2 + (d_i_3-d_avg)**2 + (d_i_4-d_avg)**2)

                marker_distortions.append(v_i)
                total_distortion += v_i

                d_i_k = abs(math.dist([cX, cY], [topLeft[0], topLeft[1]]))
                # print(d_i_k)

                # Find Pixel Area
                a_i = cv2.contourArea(corners[i])
                areas.append(a_i)
                total_area += a_i

                # Minimum Distance, choose reference marker
                if min_d > d: 
                    min_d = d
                    ref_id = ids[i][0] # set ref ID as the ID with the minimum d value

                # Draw Axis
                cv2.drawFrameAxes(frame, mtx, None, rvec, tvec, 4, 1)
                
                # Draw Border and center
                aruco_display(corners, ids, rejected_img_points, frame)

        # print(f"Ref ID: {ref_id}")

        # Using Given Geometric Relationship as denoted by the map to calculate transformation between markers
        total_weight = 0
        weights = []
        transform_matrices = []
        
        for i in range(0, len(ids)):
            # Transformation b/t Each Marker and the Reference
            t_diff_i = np.array(marker_locations[ref_id]) - np.array(marker_locations[ids[i].item()])
            print(f'Ref: {ref_id}, Current: {ids[i].item()}, Diff: {t_diff_i}')

            T_i = np.zeros([4,4])
            T_i[:3, :3] = rot_matrices[i]
            T_i[:3, 3:4] = tvecs[i]
            T_i[3, 3] = 1
            print(f'T_i: {T_i}')

            T_i_k = np.array([[1, 0, 0, t_diff_i[0]], 
                              [0, 1, 0, t_diff_i[1]], 
                              [0, 0, 1, 0], 
                              [0, 0, 0, 1]])
            print(f'T_i_k: {T_i_k}')

            # Multiply T_i by the relative matrix between current marker and the reference marker
            T_i_new = T_i @ np.linalg.inv(T_i_k) 
            print(f'T_i_new: {T_i_new}')
            transform_matrices.append(T_i_new)

            # Calculate Errors
            v_i_weight = marker_distortions[i]/total_distortion
            a_i_weight = areas[i]/total_area

            w_i = 0
            if v_i != 0: 
                w_i = a_i_weight/v_i_weight
            else: 
                w_i = a_i_weight

            weights.append(w_i)
            total_weight += w_i
        
        # Calculate Final Transformation Errors
        errors = []
        T_final = np.zeros([4,4])

        for i in range(0, len(ids)):
            error_i = weights[i]/total_weight
            errors.append(error_i)
            T_final += transform_matrices[i]*error_i
            
        # Calculate Final T''
        T_final = T_final/len(ids)
        print(f'Final: {T_final}')
        # print(f'Errors: {errors}')

        # Get Translation Vector
        pos = T_final[:3, 3]

        # Display Position on Screen
        cv2.putText(frame, f"X: {round(pos[0], 2)}, Y:{round(pos[1], 2)}", (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA, False)

        return [pos[0], pos[1]], frame
    
    else: 
        # Trigger error
        # Write error sequence 
        print("Moving Too fast please slow down")
        return [None, None], frame


def plot_chart(time, raw_x, raw_y, x_data, y_data):
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4)
    ax1.plot(time, raw_x)
    ax2.plot(time, raw_y)
    ax3.plot(time, x_data)
    ax4.plot(time, y_data)
    ax1.set_title('Raw X')
    ax2.set_title('Raw Y')
    ax3.set_title('Camera Pose X Change per Cycle (mm)')
    ax4.set_title('Camera Pose Y Change per Cycle (mm)')

    # display the plot
    plt.show()

# Find the top left point: min(x+y)
# Find the top right point: max(x-y)
# Create a straight line from the points.
# Calculate the distance of all points to the line
# If it is smaller than the radius of the circle (or a threshold): point is in the top line.
# Otherwise: point is in the rest of the block.
# Sort points of the top line by x value and save.
# Repeat until there are no points left.

# Sorting of an array of points
def sort_centers(markers, marker_size):
    markers_xy = []

    # Deep copy of the markers coordinates
    # Array to subtract points from
    searching_markers = markers[:]

    while len(searching_markers) > 0:
        # Find top left point
        top_left = sorted(searching_markers, key=lambda p: (p[0]) + (p[1]))[0]
        top_right = sorted(searching_markers, key=lambda p: (p[0]) - (p[1]))[-1]

        # tl_i = np.where(markers==top_left)[0][0]
        # tr_i = np.where(markers==top_right)[0][0]

        top_left = np.array([top_left[0], top_left[1], 0])
        top_right = np.array([top_right[0], top_right[1], 0])
        # print(f"top left: {top_left}, index: {tl_i}")
        # print(f"top right: {top_right}, index: {tr_i}")

        row = []
        remaining_markers = []

        for k in searching_markers:
            p = np.array([k[0], k[1], 0])
            index = np.where(markers==k)[0][0]
            d = marker_size  # diameter of the point of interest (threshold)
            dist = np.linalg.norm(
                np.cross(np.subtract(p, top_left), np.subtract(top_right, top_left))
            ) / np.linalg.norm(
                top_right
            )  # distance between point of interest and line a->b
            if d / 2 > (dist + 2):
                row.append(k)
            else:
                remaining_markers.append(k)

        # print(f"Row {row} \n")
        markers_xy.insert(0, sorted(row, key=lambda h: h[0]))
        searching_markers = remaining_markers

    return markers_xy

    # Find ID of each marker and put it in a matrix, this can be used to regenerate a perfect image, but we dont really need it...
    # final = [[{ids[np.where(markers==markers_xy[i][j])[0][0]][0]: markers_xy[i][j]} for j in range(0, len(markers_xy[i]))] for i in range(0, len(markers_xy))] # LOOP THROUGH
    # print(final)

    # LONGER VERSION
    # final_markers = []
    # for i in range(0, len(markers_xy)): 
    #     # print(i)
    #     row = []
    #     for j in range(0, len(markers_xy[i])): 
    #         marker = markers_xy[i][j]
    #         marker_id = ids[np.where(markers==marker)[0][0]]
    #         obj = {marker_id[0]: marker}
    #         row.append(obj)
    #         # row.append({ids[(np.where(markers==markers_xy[i][j])[0][0])]: markers_xy[i][j]})

    #     final_markers.append(row)

# 2D Rotation Matrix
def rotationMatrix2D(center, theta): 
    alpha = math.cos(theta)
    beta = math.sin(theta)

    row_1 = [alpha, beta, ((1-alpha)*center[0]-beta*center[1]).item()]
    row_2 = [-beta, alpha, (beta*center[0]+(1-alpha)*center[1]).item()]

    M = np.array([row_1, row_2])
    # print(M)
    
    return M


def manual_analyze_stitched(): 
    distance = 28.40 # mm
    dict_xy = {4: [0, 0], 3: []}
    columns = 5
    rows = 7

    ID = 4

    for i in range(0, rows): 
        for j in range(0, columns): 
            print(ID)
            
            dict_xy[ID] = [j*distance, i*distance]
            
            ID = ID - 1
        
        ID = ID + 10

    print(dict_xy)

    return dict_xy

if __name__ == '__main__':
    # analyze_stitched("result.jpg", 25.41)
    manual_analyze_stitched()