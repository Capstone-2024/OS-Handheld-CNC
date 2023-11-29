from threading_utils import WebcamVideoStream
from cv_utils import pose_estimation, plot_chart, manual_analyze_stitched, access_map, svg_to_points
import cv2
import numpy as np
import time
import pandas as pd
from kalman_utils import PE_filter
from sys import platform
# from serial_utils import ardu_write, ardu_read
from serial_utils import ArduinoComms
import math
import os
import struct


def vision_main(shape):
    # Analyze Stitched Image, establishing global coordinate system
    marker_locations = manual_analyze_stitched()

    # data_dir = os.path.abspath("./data")
    # marker_locations = access_map(data_dir)

    ''' SET UP '''
    # Target Point vars
    point_i = 0 # index of target point 

    # Plot vars and config
    x_data = []
    y_data = []
    raw_x = []
    raw_y = []
    num_data_p = 500

    # Start Camera Stream and FPS Counter
    vs = WebcamVideoStream(src=0).start()

    # FPS Counter
    prev_frame_time = 0  # used to record the time when we processed last frame
    new_frame_time = 0  # used to record the time at which we processed current frame

    # Kalman Filter
    dt = 1 / 60  # 60 fps, i want to make this dynamic but idk if it works that way
    # P
    P_x = np.diag([1**2.0, 10**2])  # covariance matrix
    P_y = np.diag([1**2.0, 10**2])

    # R
    R_x = np.array([1**2])
    R_y = np.array([1**2])

    # Q
    Q = 10**2  # process variance

    x = np.array([0.0, 0.0])
    kf_x = PE_filter(x, P_x, R_x, Q, dt)
    kf_y = PE_filter(x, P_y, R_y, Q, dt)

    # Initialize Communication with Arduino
    arduino = ArduinoComms()
    arduino.start_transmit()
    arduino.ardu_write(b'H')
    time.sleep(3)
    
    # Main Loop
    while True:
        frame = vs.read()

        # arduino.ardu_write(b'A')
        accelerometer_data = arduino.data
        print(accelerometer_data)

        ''' Calculate Position with Pose Estimation '''
        (x_pos, y_pos), output = pose_estimation(frame, marker_locations)

        print(f'Frame Size: {frame.shape[0], frame.shape[1]}')
        
        if x_pos or y_pos != None: 

            # manual_offset = [marker_locations[17][0], marker_locations[17][1]] # Should only be in x or y, this is the position of the middle marker 
            manual_offset = [50, 200]
            x_pos = x_pos + manual_offset[0]
            y_pos = y_pos + manual_offset[1]

            # Kalman Filter Predict and Update
            kf_x.predict()
            kf_x.update(x_pos)
            # print(f"X: {kf_x.x}")

            kf_y.predict()
            kf_y.update(y_pos)
            # print(f"Y: {kf_y.x}")

            ''' Calculate Local Machine Error Vector and Send to Arduino '''
            
            # pos_diff = [shape[0][point_i] - x_pos + manual_offet[0], shape[1][point_i] - y_pos + manual_offet[1]]
            # pos_diff = [shape[0][point_i] - kf_x.x[0][0] + manual_offet[0], shape[1][point_i] - kf_y.x[0][0] + manual_offet[1]]

            pos_diff = [shape[0][point_i] - kf_x.x[0], shape[1][point_i] - kf_y.x[0]]
            print(f'Vector: {pos_diff[0], pos_diff[1]}')
            
            # Send to arduino 
            if abs(pos_diff[0]) < 5 and abs(pos_diff[1]) < 5: 
                print('Sending to Arduino...')
                # arduino.ardu_write('A'.encode('ascii'))
                write_data = 'I' + (str(round(float(pos_diff[0]), 3))) + ',' + (str(round(float(pos_diff[1]), 3)))
                # arduino.ardu_write(struct.pack('>B', write_data))
                arduino.ardu_write(write_data.encode())
                print(f'Data Sent: {write_data}')
                # arduino.ardu_write(write_data.encode('ascii'))

            ''' Testing '''
            # print(f'Gloabl distance to point {point_i} is X:{pos_diff[0]} and Y: {pos_diff[1]}')
            # print(f'Local distance to point {point_i} is X:{local[0]} and Y: {local[1]}')
            print(f'Current Global: {x_pos}, {y_pos} \n')
            print(f'Filtered Global: {kf_x.x}, {kf_y.x} \n')
            
            # print(f'Target Global: {shape[0][point_i]},{shape[1][point_i]}')

            if abs(pos_diff[0]) < 2 and abs(pos_diff[1]) < 5: 
                point_i += 1


            ''' Testing Pose Estimation and Plotting for Kalman Filter '''
            ''' Collect Data for Plotting '''
            raw_x.append(float(x_pos))
            raw_y.append(float(y_pos))

            x_data.append(kf_x.x[0])
            y_data.append(kf_y.x[0])

            ''' Plot '''
            # if len(x_data) >= num_data_p:
            #     # print(record_data)
            #     # print(raw_x)
            #     plot_chart(
            #         [i for i in range(0, num_data_p)], raw_x, raw_y, x_data, y_data
            #     )
            #     df = pd.DataFrame([x_data, y_data])
            #     df.to_excel("output.xlsx")
            #     break
            

            
            ''' Calculate FPS and Display ''' 
            new_frame_time = time.time()
            fps = 1 / (new_frame_time - prev_frame_time)
            prev_frame_time = new_frame_time
            fps = int(fps)
            fps = str(fps)
            font = cv2.FONT_HERSHEY_PLAIN
            
            print(f"FPS: {fps}")
            
            ''' Only display if we are using PC '''
            if platform != "linux":
                # cv2.putText(frame, fps, (7, 70), font, 1, (100, 255, 0), 3, cv2.LINE_AA)
                # imS = cv2.resize(output, (640, 480))  
                # cv2.imshow("Output Result", imS)
                cv2.imshow("Output Result", output)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

    cv2.destroyAllWindows()
    vs.stop()


if __name__ == "__main__":
    px_to_mm = 300/100
    circle = svg_to_points('./svg_files/circle.svg', px_to_mm)
    vision_main(circle)