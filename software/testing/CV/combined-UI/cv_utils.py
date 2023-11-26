import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
from pytagmapper_tools.make_aruco_tag_txts import *
from pytagmapper_tools.build_map import *
from pytagmapper_tools.show_map import *
from sys import platform

from threading_utils import WebcamVideoStream
from svgpathtools import svg2paths

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
    corners = []
    ids = []
    
    if platform == "linux": # cv2 V4.5
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    else:  # cv2 V4.7+
        dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)

        corners, ids, _ = detector.detectMarkers(gray)

    ''' Pose Estimation (Most computing heavy) '''
    if len(corners) > 0:

        # Get an array of all available markers
        available_ids = marker_locations.keys()
        
        # Reference Marker Vars
        ref_id = ids[0]
        min_d = float('inf') # arbitrarily large distance
        ref_corners = []

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
                
                # mtx, roi = cv2.getOptimalNewCameraMatrix(matrix_coefficients, distortion_coefficients, (frame.shape[0], frame.shape[1]), 0.5, (frame.shape[0], frame.shape[1]))
                # ret, rvec, tvec = cv2.solvePnP(objp, corners[i], mtx, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                ret, rvec, tvec = cv2.solvePnP(objp, corners[i], matrix_coefficients, distortion_coefficients, flags=cv2.SOLVEPNP_IPPE_SQUARE)

                # Add Rotation Matrix and Translation Vector
                rot_matrices.append(cv2.Rodrigues(rvec)[0])
                tvecs.append(tvec)

                # Find Marker Center
                (topLeft, topRight, bottomRight, bottomLeft) = corners[i][0]
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)

                # Image Center
                image_cX = int(frame.shape[1]/2.0)
                image_cY = int(frame.shape[0]/2.0)

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
                    ref_corners = corners[i][0]

                # Draw Axis
                # cv2.drawFrameAxes(frame, mtx, None, rvec, tvec, 4, 1)
                cv2.drawFrameAxes(frame, matrix_coefficients, None, rvec, tvec, 4, 1)
                
                # Draw Border and center
                aruco_display(corners, ids, rejected_img_points, frame)

        # print(f"Ref ID: {ref_id}")

        # Using Given Geometric Relationship as denoted by the map to calculate transformation between markers
        total_weight = 0
        weights = []
        transform_matrices = []
        global_pos_data = {}
        
        for i in range(0, len(ids)):
            # Transformation b/t Each Marker and the Reference
            t_diff_i = np.array(marker_locations[ref_id]) - np.array(marker_locations[ids[i].item()])
            # print(f'Ref: {ref_id}, Current: {ids[i].item()}, Diff: {t_diff_i}')

            T_i = np.eye(4)
            T_i[:3, :3] = rot_matrices[i]
            T_i[:3, 3:4] = tvecs[i]
            # print(f'T_i: {T_i}')
            global_pos_data[ids[i].item()] = [tvec[0], tvec[1]]

            # T_i_k = np.eye(4)
            # T_i_k[:3, 3] = np.transpose(t_diff_i)
            T_i_k = np.array([[1, 0, 0, t_diff_i[0]], 
                              [0, 1, 0, t_diff_i[1]], 
                              [0, 0, 1, 0], 
                            #   [0, 0, 1, t_diff_i[2]], 
                              [0, 0, 0, 1]])
            # print(f'T_i_k: {T_i_k}')

            # Multiply T_i by the relative matrix between current marker and the reference marker
            # Quick Inversion 
            # T_i_k_inv = np.eye(4)
            # T_i_k_inv[:3, :3] = T_i_k[:3, :3]
            # T_i_k_inv[:3, 3:4] = -T_i_k[:3, :3] @ T_i_k[:3, 3:4]

            # T_i_new = T_i @ T_i_k_inv
            T_i_new = T_i @ np.linalg.inv(T_i_k) 

            # print(f'ID: {ids[i].item()} \nT_i_new: {T_i_new}')
            transform_matrices.append(np.linalg.inv(T_i_new))

            # Calculate Errors
            v_i_weight = marker_distortions[i]/total_distortion
            a_i_weight = areas[i]/total_area

            w_i = 0
            if v_i != 0: 
                w_i = a_i_weight/v_i_weight
            else: 
                w_i = a_i_weight/0.00001

            weights.append(w_i)
            total_weight += w_i

        # Plot all transformation matrices camera pos
        # markers = global_pos_data.items() # sorted by key, return a list of tuples
        # l, xy= zip(*markers) # unpack a list of pairs into two tuples
        # x, y = zip(*xy)
        # fig, ax = plt.subplots()
        # ax.scatter(x, y)
        # ax.legend(l)
        # plt.show()
        
        # Calculate Final Transformation Errors
        T_final = np.zeros([4,4])

        for i in range(0, len(ids)):
            error_i = weights[i]/total_weight
            T_final = np.add(T_final, error_i*transform_matrices[i])
            # print(f'ID: {ids[i].item()}, error: {error_i}')
            
        # Calculate Final T''
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

def access_map(data_dir): 
    if len(glob.glob("./data/*.png")) == 0:
        # Image taking sequence
        # Start Camera Stream and FPS Counter
        vs = WebcamVideoStream(src=0).start()

        i = 0
        while True:
            frame = vs.read()

            cv2.imshow("frame", frame)

            if chr(cv2.waitKey(1) & 255) == "c":  # capture key, change to button later
                cv2.imwrite("./data/image_" + str(i) + ".png", frame)

                i += 1
                print("Photo saved... Press 'q' to exit or continue taking photos. \n")

            elif chr(cv2.waitKey(1) & 255) == "q":  # Change to button click later
                break

        cv2.destroyAllWindows()
        vs.stop()

        print("Capture finished. Now building map ... Please wait. \n")

    """ Build Map """
    # Build Aruco Txt Data Files
    # Fixed hard coded path
    if len(glob.glob("./data/*.txt")) <= 1: 
        aruco_tag_data(data_dir, False)

    if len(glob.glob("./data/*.json")) == 0:
        build_map(data_dir, data_dir, "2d")
        # Display some stuff about the map - maybe size or whatever
        print("Finished. \n")

    show_map(data_dir)
    
    map_data = load_map("data")
    
    return map_data['tag_locations']

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
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}

def aruco_display(corners, ids, rejected, image, terminal_print=False):
    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for markerCorner, markerID in zip(corners, ids):
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
            cv2.putText(
                image,
                str(markerID),
                (topLeft[0], topLeft[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )
            if terminal_print:
                print("[Inference] ArUco marker ID: {}".format(markerID))
            # show the output image
    return image

def svg_to_points(svg_file, ratio, num_samples=5, view=False):
    print("Converting to points.")
    # Convert SVG file to paths
    paths, attributes = svg2paths(svg_file)

    # Extract data points from paths
    x_coord = []
    y_coord = []
    for path in paths:
        for segment in path:
            length = segment.length()
            num_segments = int(length)  # Number of line segments to approximate the line
            t_values = [i / num_segments for i in range(num_segments + 1)]
            for t in t_values:
                point = segment.point(t)
                x_coord.append(point.real)
                y_coord.append(point.imag)
        
    # Flip Y
    flip_y = [y - max(y_coord) for y in y_coord]

    # plot_x = [ x-min(x_coord) for x in x_coord]
    # plot_y = [ y-flip_y[x_coord.index(min(x_coord))] for y in flip_y]

    x_final = [i/ratio for i in x_coord]
    y_final = [i/ratio for i in y_coord]

    if view: 
        plt.scatter(x_final, y_final)
        plt.show()

    print("DONE")

    return x_final, y_final

if __name__ == '__main__':
    # analyze_stitched("result.jpg", 25.41)
    manual_analyze_stitched()