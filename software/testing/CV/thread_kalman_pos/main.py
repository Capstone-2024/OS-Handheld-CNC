from threading_utils import WebcamVideoStream, ARUCO_DICT
from cv_utils import analyze_stitched, pose_estimation, plot_chart
import cv2
import numpy as np
import time
import pandas as pd
from kalman_utils import PE_filter
from sys import platform
from serial_utils import ardu_read


def main():
    # Analyze Stitched Image, establishing global coordinate system
    marker_locations = analyze_stitched(img_path="result.jpg", marker_size=25)

    # Record camera pose relative to each marker according to their unique id, using a dictionary
    current_pose = {}
    prev_pose = {}

    # Plot characteristics
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
    P_x = np.diag([0.5**2.0, 5**2])  # covariance matrix
    P_y = np.diag([0.5**2.0, 5**2])

    # R
    R_x = np.array([0.5**2])
    R_y = np.array([0.5**2])

    # Q
    Q = 0.5**2  # process variance

    x = np.array([0.0, 0.0])
    kf_x = PE_filter(x, P_x, R_x, Q, dt)
    kf_y = PE_filter(x, P_y, R_y, Q, dt)





    # Main Loop
    while True:
        frame = vs.read()

        # Read Arduino for safety checks or status updates
        data = ardu_read()
        if data is not 0: 
            # do something
            print(data)

            # Handle whatever data says here



        # CV Stuff

        try:  # prevent errors
            (x_pos, y_pos), output = pose_estimation(frame, marker_locations, current_pose, prev_pose)

            raw_x.append(float(x_pos))
            raw_y.append(float(y_pos))

            # Kalman Filter Predict and Update
            kf_x.predict()
            kf_x.update(x_pos)
            print(f"X: {kf_x.x}")

            kf_y.predict()
            kf_y.update(y_pos)
            print(f"Y: {kf_y.x}")

            x_data.append(kf_x.x[0])
            y_data.append(kf_y.x[0])

            if len(x_data) >= num_data_p:
                # print(record_data)
                print(raw_x)
                plot_chart(
                    [i for i in range(0, num_data_p)], raw_x, raw_y, x_data, y_data
                )
                df = pd.DataFrame([x_data, y_data])
                df.to_excel("output.xlsx")
                break
            
            # FPS 
            new_frame_time = time.time()
            fps = 1 / (new_frame_time - prev_frame_time)
            prev_frame_time = new_frame_time
            fps = int(fps)
            fps = str(fps)
            font = cv2.FONT_HERSHEY_PLAIN
            print(f"FPS: {fps}")

            # cv2.putText(frame, fps, (7, 70), font, 1, (100, 255, 0), 3, cv2.LINE_AA)

            if platform != "linux":
                cv2.imshow("Output Result", output)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

        except:
            print("No markers found")

            if len(x_data) >= num_data_p:
                break

    cv2.destroyAllWindows()
    vs.stop()


if __name__ == "__main__":
    main()
