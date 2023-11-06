from threading_utils import WebcamVideoStream, ARUCO_DICT, aruco_display
import cv2
import numpy as np
import time
import pandas as pd
import matplotlib.pyplot as plt
import math
from kalman_utils import PE_filter
from sys import platform


def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients, current_pose, prev_pose):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # more processing can be done to the images
    
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
            marker_size = 25 # mm
            
            # Object points
            objp = np.array([[-marker_size / 2, marker_size / 2, 0],
                                    [marker_size / 2, marker_size / 2, 0],
                                    [marker_size / 2, -marker_size / 2, 0],
                                    [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)

            ret, rvec, tvec = cv2.solvePnP(objp, corners[i], matrix_coefficients, distortion_coefficients, False, cv2.SOLVEPNP_IPPE_SQUARE)
            
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
                # DONT DO THIS, we should be avearging the change in position, but does this mean we need a kalman filter for each marker? 
                x_sum = x_sum + value['translation'][0]
                y_sum = y_sum + value['translation'][1]

            x_pos = x_sum/len((ids))
            y_pos = y_sum/len((ids))

            # Draw Axis
            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 4, 1)

            # increment counter
            i += 1

            aruco_display(corners, ids, rejected_img_points, frame)

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

def stream(): 
    aruco_dict_type = ARUCO_DICT["DICT_6X6_250"]

    # Record camera pose relative to each marker according to their unique id, using a dictionary
    current_pose = {}
    prev_pose = {}

    # Plot characteristics
    x_data = []
    y_data = []
    raw_x = []
    raw_y = []
    num_data_p = 500
    
    # load numpy data files
    k = np.load("./calibration_matrix.npy")
    d = np.load("./distortion_coefficients.npy")

    # Start Camera Stream and FPS Counter
    vs = WebcamVideoStream(src=0).start()

    # used to record the time when we processed last frame
    prev_frame_time = 0
    
    # used to record the time at which we processed current frame
    new_frame_time = 0

    # sample_time = 0.05 # second
    # prev_sample_time = 0 

    # Kalman Filter
    dt = 1/60 # 60 fps, i want to make this dynamic but idk if it works that way
    # P
    P_x = np.diag([0.5**2., 5**2]) # covariance matrix
    P_y = np.diag([0.5**2., 5**2])

    # R 
    R_x = np.array([0.5**2])
    R_y = np.array([0.5**2])

    # Q
    Q = 0.5**2 # process variance

    x = np.array([0., 0.])
    kf_x = PE_filter(x, P_x, R_x, Q, dt)
    kf_y = PE_filter(x, P_y, R_y, Q, dt)

    # Main Loop
    while True:

        frame = vs.read()

        # try: # prevent errors
        (x_pos, y_pos), output = pose_estimation(frame, aruco_dict_type, k, d, current_pose, prev_pose)

        raw_x.append(float(x_pos))
        raw_y.append(float(y_pos))

        kf_x.predict()
        kf_x.update(x_pos)
        print(f'X: {kf_x.x}')

        kf_y.predict()
        kf_y.update(y_pos)
        print(f'Y: {kf_y.x}')

        x_data.append(kf_x.x[0])
        y_data.append(kf_y.x[0])

        if len(x_data) >= num_data_p: 

            # print(record_data)
            print(raw_x)
            plot_chart([i for i in range(0, num_data_p)], raw_x, raw_y, x_data, y_data)
            df = pd.DataFrame([x_data, y_data])
            df.to_excel('output.xlsx')
            break
    
        # if change: 
        #     hyp = math.sqrt(change[0]**2 + change[1]**2)
        #     record_data.append(hyp)
        #     print(hyp)
        
        # if change: 
        # if change: 
        #     record_data.append(change)
        # hyp = math.sqrt(change[0]**2 + change[1]**2)

        # if change: 
        #     record_data.append(hyp)
        # print(hyp)

        # if len(record_data) >= num_data_p: 

        #     print(record_data)
        #     velocity = [d/sample_time for d in record_data]
        #     plot_chart([i for i in range(0, num_data_p)], record_data, velocity)
        #     # df = pd.DataFrame(record_data)
        #     # df.to_excel('output.xlsx')
        #     break

        new_frame_time = time.time()
        fps = 1/(new_frame_time-prev_frame_time)
        prev_frame_time = new_frame_time
        fps = int(fps)
        fps = str(fps)
        font = cv2.FONT_HERSHEY_PLAIN
        print(f'FPS: {fps}')

        cv2.putText(frame, fps, (7, 70), font, 1, (100, 255, 0), 3, cv2.LINE_AA)

        if platform != "linux":
            cv2.imshow('Output Result', output) 

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        
        # except: 
        #     print("No markers found")

        #     if len(x_data) >= num_data_p: 
        #         break

    cv2.destroyAllWindows()
    vs.stop() 


if __name__ == '__main__': 
    stream()