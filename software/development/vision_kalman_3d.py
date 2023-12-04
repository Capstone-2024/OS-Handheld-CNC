from threading_utils import WebcamVideoStream
from cv_utils import pose_estimation, plot_chart, manual_analyze_stitched, access_map, svg_to_points, rotationMatrix2D
import cv2
import numpy as np
import time
import pandas as pd
from kalman_utils_3d import PE_filter
from sys import platform
from serial_utils import ArduinoComms

def vision_main(shape):
    ''' Main Vision Program for Pose Estimation '''

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
    P_x = np.diag([1**2.0, 10**2, 10**2])  # covariance matrix
    P_y = np.diag([1**2.0, 10**2, 10**2])

    # R - Measurement Error
    R_x = np.array([1**2, 20**2])
    R_y = np.array([1**2, 20**2])

    # Q - process variance
    Q = 10**2

    x = np.array([0.0, 0.0, 0.0])
    kf_x = PE_filter(x, P_x, R_x, Q, dt)
    kf_y = PE_filter(x, P_y, R_y, Q, dt)

    # Initialize Communication with Arduino
    arduino = ArduinoComms()

    while arduino.homingOperation() != 'G': 
        # print(arduino_communicator.link.rxBuff)
        time.sleep(0.5)
        continue

    while arduino.zHoming() != 'O': 
        time.sleep(0.5)
        continue

    num_accel_samples = 10
    offsets_x = np.zeros(num_accel_samples)
    offsets_y = np.zeros(num_accel_samples)
    
    # Accelerometer offsets
    for i in range(0, num_accel_samples): 
        _, x, y = arduino.regOperation()
        offsets_x[i] = x
        offsets_y[i] = y

    accel_offset_x = np.average(offsets_x)
    accel_offset_y = np.average(offsets_y)

    # Main Loop
    while True:
        frame = vs.read()

        # Read Accelerometer
        status, accel_x, accel_y = arduino.regOperation()
        accel_x_mm = (accel_x - accel_offset_x)*1000
        accel_y_mm = (accel_y - accel_offset_y)*1000

        ''' Calculate Position with Pose Estimation '''
        (x_pos, y_pos), z_rot, output = pose_estimation(frame, marker_locations)
        
        # Make sure all data are available
        if x_pos or y_pos or z_rot != None and status == 'Y': 

            # manual_offset = [marker_locations[17][0], marker_locations[17][1]] # Should only be in x or y, this is the position of the middle marker 
            manual_offset = [50, 200]
            x_pos = x_pos + manual_offset[0]
            y_pos = y_pos + manual_offset[1]

            # Kalman Filter Predict and Update
            kf_x.predict()
            kf_x.update([x_pos, accel_x_mm])
            # print(f"X: {kf_x.x}")

            kf_y.predict()
            kf_y.update([y_pos, accel_y_mm])
            # print(f"Y: {kf_y.x}")

            ''' Calculate Local Machine Error Vector and Send to Arduino '''
            
            # pos_diff = [shape[0][point_i] - kf_x.x[0][0] + manual_offet[0], shape[1][point_i] - kf_y.x[0][0] + manual_offet[1]]

            # pos_diff = [shape[0][point_i] - kf_x.x[0], shape[1][point_i] - kf_y.x[0]]

            ''' Fixed At One Point '''
            test_point = [0, 0]
           
            pos_diff = [test_point[0] - kf_x.x[0], test_point[1] - kf_y.x[0]]
            print(f'Global Vector: {pos_diff[0], pos_diff[1]}')

            # Z Rotation About 0, 0
            # rot_M = rotationMatrix2D([0, 0], z_rot)
            # loc_diff = rot_M @ np.array([[pos_diff[0]],[pos_diff[1]],[0]]) # apply rotation in the vector sent since the coordinate is relative to the local
            # print(f'Local Vector: {loc_diff[0], loc_diff[1]}')

            # Send to arduino 
            # if abs(loc_diff[0]) < 5 and abs(loc_diff[1]) < 5: 
            #     print('Sending to Arduino...')
            #     arduino.send_error(loc_diff[0], loc_diff[1])
            #     print(f'Data Sent: {loc_diff[0]}, {loc_diff[1]}')

            if abs(pos_diff[0]) <= 5 and abs(pos_diff[1]) <= 5: 
                print('Sending to Arduino...')
                arduino.send_error(pos_diff[0], pos_diff[1])
                print(f'Data Sent: {pos_diff[0]}, {pos_diff[1]}')

            ''' Testing '''
            # print(f'Gloabl distance to point {point_i} is X:{pos_diff[0]} and Y: {pos_diff[1]}')
            # print(f'Local distance to point {point_i} is X:{local[0]} and Y: {local[1]}')
            print(f'Current Global: {x_pos}, {y_pos}')
            print(f'Filtered Global: {kf_x.x[0]}, {kf_y.x[0]} \n')
            
            # print(f'Target Global: {shape[0][point_i]},{shape[1][point_i]}')

            # if abs(pos_diff[0]) < 5 and abs(pos_diff[1]) < 5: 
            #     point_i += 1
            # print(f'Step: {point_i}')

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
            # new_frame_time = time.time()
            # fps = 1 / (new_frame_time - prev_frame_time)
            # prev_frame_time = new_frame_time
            # fps = int(fps)
            # fps = str(fps)
            # font = cv2.FONT_HERSHEY_PLAIN
            
            # print(f"FPS: {fps}")
            
            ''' Only display if we are using PC '''
            if platform != "linux":
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