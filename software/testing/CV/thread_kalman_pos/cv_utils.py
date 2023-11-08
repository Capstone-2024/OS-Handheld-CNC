from threading_utils import ARUCO_DICT, aruco_display
import cv2
import numpy as np
import time
import pandas as pd
import matplotlib.pyplot as plt
import math
from sys import platform
import subprocess

# Generate a matrix of all the markers
def analyze_stitched(img_path, marker_size):
    # Load Data
    aruco_dict_type = ARUCO_DICT["DICT_6X6_250"]
    matrix_coefficients = np.load("./calibration_matrix.npy")
    distortion_coefficients = np.load("./distortion_coefficients.npy")
    
    # Find all the markers
    img = cv2.imread(img_path)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    corners, ids, rejected_img_points = detector.detectMarkers(gray)


    # Numpy array of markers
    world_markers_xy = np.zeros(shape=(len(ids), 2))
    image_markers_xy = np.zeros(shape=(len(ids), 2))


    # If markers are detected
    if len(ids) > 0:

        for i in range(0, len(ids)):
            # Size of the marker in real life in mmm
            marker_size = 25  # mm

            # Object points
            objp = np.array([[-marker_size / 2, marker_size / 2, 0],
                             [marker_size / 2, marker_size / 2, 0],
                             [marker_size / 2, -marker_size / 2, 0],
                             [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)

            ret, rvec, tvec = cv2.solvePnP(
                objp, corners[i], matrix_coefficients, distortion_coefficients, False, cv2.SOLVEPNP_IPPE_SQUARE)

            world_markers_xy[i] = [tvec[0][0], tvec[1][0]]
            # print(corners[i])

            (topLeft, _, bottomRight, _) = corners[i][0]
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)

            image_markers_xy[i] = [cX, cY]

    else:
        return None

    # Plots for visualization of the markers' coordinates in world and image frames
    world_coord_data = world_markers_xy.T
    image_coord_data = image_markers_xy.T
    x1, y1 = world_coord_data
    x2, y2 = image_coord_data

    fig, (ax1, ax2) = plt.subplots(2)
    ax1.scatter(x1, y1)
    ax1.set_title("Pose Estimation Marker Center World Coordinates")
    ax2.scatter(x2, y2)
    ax2.set_title("Marker Center Image Coordinates")
    plt.show()

    # Sort markers and make rows
    sorted_xy = sort_centers(world_markers_xy, marker_size, ids)
    # print(sorted_xy)

    df = pd.DataFrame(sorted_xy)
    df.style \
        .format(precision=3) \
        .format_index(str.upper, axis=1)
    print(df)
    # return marker_matrix


def pose_estimation(frame, current_pose, prev_pose):
    # Pose Estimation Initialize 
    aruco_dict_type = ARUCO_DICT["DICT_6X6_250"]
    matrix_coefficients = np.load("./calibration_matrix.npy")
    distortion_coefficients = np.load("./distortion_coefficients.npy")

    # more processing can be done to the images
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    corners, ids, rejected_img_points = detector.detectMarkers(gray)

    # Marker Detection and Pose Estimation
    # Most computing heavy
    if len(corners) > 0:

        # Store Sum of Differences
        cam_change = [0, 0, 0]

        i = 0
        # For each detected ID
        for i in range(0, len(ids)):

            # Estimate pose of each marker and return the camera's rotational and translational vectors

            # Size of the marker in real life in mmm
            marker_size = 25  # mm

            # Object points
            objp = np.array([[-marker_size / 2, marker_size / 2, 0],
                             [marker_size / 2, marker_size / 2, 0],
                             [marker_size / 2, -marker_size / 2, 0],
                             [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)

            ret, rvec, tvec = cv2.solvePnP(
                objp, corners[i], matrix_coefficients, distortion_coefficients, False, cv2.SOLVEPNP_IPPE_SQUARE)

            # XY rotation might not be useful since it will be fixed
            # rotation around Z might be useful, since it can show us where the camera is pointing

            # If marker has been stored
            # if str(ids[i][0]) in current_pose:
            #     # Store Current Pos
            #     prev_pose[str(ids[i][0])] = current_pose[str(ids[i][0])]

            #     # Calculate the Change in Translation for each marker
            #     difference = tvec - prev_pose[str(ids[i][0])]['translation']
            #     # print("Marker {} moved by: X: {}, Y: {}, Z: {}".format(ids[i][0], difference[0], difference[1], difference[2]))

            #     # Add marker's change into average sum
            #     # for j in range(0, len(tvec)):
            #     #     cam_change[j] += difference[j] # Positive

            # Overwrite/add pose
            # current_pose[str(ids[i][0])] = {'rotation': rvec,'translation': tvec}
            # print(current_pose)

            current_pose[str(ids[i][0])] = {'translation': tvec}

            x_sum = 0
            y_sum = 0
            x_pos = 0
            y_pos = 0

            for key, value in current_pose.items():
                # DONT DO THIS, we should be averaging the change in position, but does this mean we need a kalman filter for each marker?
                x_sum = x_sum + value['translation'][0]
                y_sum = y_sum + value['translation'][1]

            x_pos = x_sum/len((ids))
            y_pos = y_sum/len((ids))

            # Draw Axis
            cv2.drawFrameAxes(frame, matrix_coefficients,
                              distortion_coefficients, rvec, tvec, 4, 1)

            # increment counter
            i += 1

            # aruco_display(corners, ids, rejected_img_points, frame)

            return [x_pos, y_pos], frame

        # Return Change
        # if cam_change:
        # # Add Data Tag
        #     # aruco_display(corners, ids, rejected_img_points, frame)
        #     return [cam_change[0]/len(ids), cam_change[1]/len(ids)], frame
        # else:
        #     return [0, 0], frame


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
def sort_centers(markers, marker_size, ids):
    markers_xy = []
    final_markers = {}

    # Deep copy of the markers coordinates
    # Array to subtract points from
    searching_markers = markers[:]

    while len(searching_markers) > 0:
        # Find top left point
        top_left = sorted(searching_markers, key=lambda p: (p[0]) + (p[1]))[0]
        top_right = sorted(searching_markers, key=lambda p: (p[0]) - (p[1]))[-1]

        tl_i = np.where(markers==top_left)[0][0]
        tr_i = np.where(markers==top_right)[0][0]

        top_left = np.array([top_left[0], top_left[1], 0])
        top_right = np.array([top_right[0], top_right[1], 0])
        print(f"top left: {top_left}, index: {tl_i}")
        print(f"top right: {top_right}, index: {tr_i}")

        row = []
        remaining_markers = []

        for k in searching_markers:
            p = np.array([k[0], k[1], 0])
            index = np.where(markers==k)[0][0]
            d = marker_size  # diameter of the keypoint (might be a theshold)
            dist = np.linalg.norm(
                np.cross(np.subtract(p, top_left), np.subtract(top_right, top_left))
            ) / np.linalg.norm(
                top_right
            )  # distance between keypoint and line a->b
            if d / 2 > (dist + 2):
                row.append(k)
            else:
                remaining_markers.append(k)

        print(f"Row {row} \n")
        markers_xy.append(sorted(row, key=lambda h: h[0]))
        searching_markers = remaining_markers

        # find ID of each marker and put it in a matrix
        row = [[{ids[np.where(markers==markers[i])[0][0]]: markers[i]} for i in range(num_cols)] for _ in range(num_rows)] # LOOP THROUGH


    return markers_xy

if __name__ == '__main__':
    # stream(

    aruco_dict_type = ARUCO_DICT["DICT_6X6_250"]
    k = np.load("./calibration_matrix.npy")
    d = np.load("./distortion_coefficients.npy")
    analyze_stitched("result.jpg", 25)