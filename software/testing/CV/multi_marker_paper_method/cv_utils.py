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
    # Dictionary for accessing the marker locations
    dict_xy = {}

    # Load Data
    aruco_dict_type = ARUCO_DICT["DICT_6X6_250"]

    # Jig 
    # matrix_coefficients = np.load("./calibration_matrix.npy")
    # distortion_coefficients = np.load("./distortion_coefficients.npy")

    # New Camera
    matrix_coefficients = np.load("./calibration_matrix_2.npy")
    distortion_coefficients = np.load("./distortion_coefficients_2.npy")
    
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
            marker_size = 25.41 # mm

            # Object points
            objp = np.array([[-marker_size / 2, marker_size / 2, 0],
                             [marker_size / 2, marker_size / 2, 0],
                             [marker_size / 2, -marker_size / 2, 0],
                             [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)

            ret, rvec, tvec = cv2.solvePnP(
                objp, corners[i], matrix_coefficients, distortion_coefficients, False, cv2.SOLVEPNP_IPPE_SQUARE)
            
            ''' Transform from Camera to World Coordinate '''
            rot_M = cv2.Rodrigues(rvec)[0]
            cameraPosition = -np.matrix(rot_M).T * np.matrix(tvec)

            world_markers_xy[i] = [cameraPosition[0].item(), -1*cameraPosition[1].item()]
            
            # world_markers_xy[i] = [tvec[0][0], -1*tvec[1][0]] #-1 to change the position according for world coordinate

            dict_xy[ids[i][0]] = world_markers_xy[i]

            (topLeft, _, bottomRight, _) = corners[i][0]
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)

            image_markers_xy[i] = [cX, cY]

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


    # Sort markers and make rows, not doing this anymore
    sorted_xy = sort_centers(world_markers_xy, marker_size)
    # print(sorted_xy)

    df = pd.DataFrame(sorted_xy)
    df.style \
        .format(precision=3) \
        .format_index(str.upper, axis=1)
    print(df)
    # df.to_csv("marker-loc.csv", index=False)

    # print(dict_xy)

    # Find Marker at the Bottom Left Corner and offset the entire matrix
    offset_xy = {}
    for id, value in dict_xy.items(): 
        offset_xy[id] = [value[0] - sorted_xy[-1][0][0], value[1] - sorted_xy[-1][0][1]]

    print(f'Final Markers Dict: {offset_xy}')
    return offset_xy


def pose_estimation(frame, marker_locations):
    # Pose Estimation Initialize 
    aruco_dict_type = ARUCO_DICT["DICT_6X6_250"]
    matrix_coefficients = np.load("./calibration_matrix.npy")
    distortion_coefficients = np.load("./distortion_coefficients.npy")

    # more processing can be done to the images
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Marker Detection
    dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    corners, ids, rejected_img_points = detector.detectMarkers(gray)

    ''' Pose Estimation (Most computing heavy) '''
    if len(corners) > 0:

        # Store Sum
        global_pos_sum = [0, 0]
        gloabl_z_rotation = 0

        # Store all data
        global_pos_data = {}

        # Get an array of all available markers
        available_ids = marker_locations.keys()

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

                ret, rvec, tvec = cv2.solvePnP(
                    objp, corners[i], matrix_coefficients, distortion_coefficients, False, cv2.SOLVEPNP_IPPE_SQUARE)

                ''' Transform from Camera to World Coordinate https://stackoverflow.com/questions/18637494/camera-position-in-world-coordinate-from-cvsolvepnp ''' 
                rot_M = cv2.Rodrigues(rvec)[0]
                cameraPosition = -np.matrix(rot_M).T * np.matrix(tvec)

                # print(cameraPosition)

                ''' Then Transform to Global Coordinate '''
                # Use TVEC, representing the relative position of each marker to the camera
                # then add by the global value to get the real value
                id_global_pos = [cameraPosition[0] - marker_locations[ids[i][0]][0], cameraPosition[1] - marker_locations[ids[i][0]][1]] # Global ID Position, 3D

                # global_pos_data[ids[i][0]] = [cameraPosition[0] - marker_locations[ids[i][0]][0], cameraPosition[1] - marker_locations[ids[i][0]][1]] # Store position in dictionary (backup/visualize)
                global_pos_data[ids[i][0]] = [cameraPosition[0], cameraPosition[1]] # Store position in dictionary (backup/visualize)

                global_pos_sum = [global_pos_sum[0] + id_global_pos[0], global_pos_sum[1] + id_global_pos[1]] # Add new marker to sum

                gloabl_z_rotation =+ rvec[2] # Add rotation about Z
                
                # Draw Axis
                cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 4, 1)
                
                # Draw Border and center
                aruco_display(corners, ids, rejected_img_points, frame)
        
        # Show Marker Pos
        # print(global_pos_data[4])
        markers = global_pos_data.items() # sorted by key, return a list of tuples
        l, xy= zip(*markers) # unpack a list of pairs into two tuples
        x, y = zip(*xy)
        fig, ax = plt.subplots()
        ax.scatter(x, y)
        ax.legend(l)
        plt.show()

        # Calculate Average of all markers
        avg_pos = [global_pos_sum[0]/len(available_ids), global_pos_sum[1]/len(available_ids)]
        # print(avg_pos)

        avg_z_rotation = gloabl_z_rotation/len(available_ids)

        return avg_pos, avg_z_rotation, frame
    
    else: 
        # Trigger error
        # Write error sequence 
        print("Moving Too fast please slow down")
        return [None, None], None, frame


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