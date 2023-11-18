# https://link.springer.com/chapter/10.1007/11941354_25

from threading_utils import WebcamVideoStream, ARUCO_DICT
from cv_utils import analyze_stitched, pose_estimation, rotationMatrix2D, plot_chart, manual_analyze_stitched
import cv2
import numpy as np
import time
import pandas as pd
from kalman_utils import PE_filter
from sys import platform
from serial_utils import ardu_write, ardu_read
import math


def main(shape):
    # Analyze Stitched Image, establishing global coordinate system
    # marker_locations = analyze_stitched(img_path="result.jpg", marker_size=25.41)
    marker_locations = manual_analyze_stitched()
    
    # Target Point vars
    point_i = 0 # index of target point 

    # Plot vars and config
    x_data = []
    y_data = []
    raw_x = []
    raw_y = []
    num_data_p = 1000

    # Start Camera Stream and FPS Counter
    vs = WebcamVideoStream(src=0).start()

    # FPS Counter
    prev_frame_time = 0  # used to record the time when we processed last frame
    new_frame_time = 0  # used to record the time at which we processed current frame

    # Kalman Filter
    dt = 1 / 60  # 60 fps, i want to make this dynamic but idk if it works that way
    # P
    P_x = np.diag([0.5**2.0, 5**2])  # covariance matrix
    P_y = np.diag([0.5**2.0, 5**2])

    # R
    R_x = np.array([0.5**2])
    R_y = np.array([0.5**2])

    # Q
    Q = 10**2  # process variance

    x = np.array([0.0, 0.0])
    kf_x = PE_filter(x, P_x, R_x, Q, dt)
    kf_y = PE_filter(x, P_y, R_y, Q, dt)

    # Main Loop
    while True:
        frame = vs.read()

        ''' Read Arduino for safety checks or status updates '''
        # data = ardu_read()
        # if data != 0: 
        #     # do something
        #     print(data)

            # Handle whatever data says here

        ''' Calculate Position with Pose Estimation '''
        (x_pos, y_pos), z_rotation, output = pose_estimation(frame, marker_locations)
        
        if x_pos or y_pos != None: 

            # Kalman Filter Predict and Update
            kf_x.predict()
            kf_x.update(x_pos)
            print(f"X: {kf_x.x}")

            kf_y.predict()
            kf_y.update(y_pos)
            print(f"Y: {kf_y.x}")

            ''' Calculate Local Machine Error Vector and Send to Arduino '''
            manual_offet = [0, 0] # Should only be in x or y
            pos_diff = [shape[0][point_i] - x_pos + manual_offet[0], shape[1][point_i] - y_pos + manual_offet[1]]
            # pos_diff = [shape[0][point_i] - kf_x.x[0][0] + manual_offet[0], shape[1][point_i] - kf_y.x[0][0] + manual_offet[1]]

            ''' Transformation matrix, rotation about Z, rotate about the current location '''
            T = rotationMatrix2D([x_pos, y_pos], z_rotation)
            local = np.matmul(T, np.array([pos_diff[0].item(), pos_diff[1].item(), 1]))
            # print(f'Transformed: {local}')
            
            # Send to arduino 
            

            ''' Testing '''
            # print(f'Gloabl distance to point {point_i} is X:{pos_diff[0]} and Y: {pos_diff[1]}')
            # print(f'Local distance to point {point_i} is X:{local[0]} and Y: {local[1]}')
            # print(f'Current Global: {x_pos}, {y_pos}')
            # print(f'Target Global: {shape[0][point_i]},{shape[1][point_i]}')

            if abs(pos_diff[0]) < 5 and abs(pos_diff[1]) < 5: 
                point_i += 1


            ''' Project Shape with points on to the frame '''
            # Use the RVEC from the pose estimation and the Z value to tranform the points to the image space



            ''' Testing Pose Estimation and Plotting for Kalman Filter '''
            ''' Collect Data for Plotting '''
            raw_x.append(float(x_pos))
            raw_y.append(float(y_pos))

            x_data.append(kf_x.x[0])
            y_data.append(kf_y.x[0])

            ''' Plot '''
            if len(x_data) >= num_data_p:
                # print(record_data)
                # print(raw_x)
                plot_chart(
                    [i for i in range(0, num_data_p)], raw_x, raw_y, x_data, y_data
                )
                df = pd.DataFrame([x_data, y_data])
                df.to_excel("output.xlsx")
                break



            

            
            ''' Calculate FPS and Display ''' 
            # new_frame_time = time.time()
            # fps = 1 / (new_frame_time - prev_frame_time)
            # prev_frame_time = new_frame_time
            # fps = int(fps)
            # fps = str(fps)
            # font = cv2.FONT_HERSHEY_PLAIN
            
            # print(f"FPS: {fps}")
            
            ''' Only display if we are using PC '''
            if platform != "linux":
                # cv2.putText(frame, fps, (7, 70), font, 1, (100, 255, 0), 3, cv2.LINE_AA)
                cv2.imshow("Output Result", output)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

    cv2.destroyAllWindows()
    vs.stop()


if __name__ == "__main__":
    main(None)
