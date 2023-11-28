import cv2
import numpy as np
import matplotlib.pyplot as plt
from sys import platform
from scipy.optimize import minimize
from cv_utils import manual_analyze_stitched

# Define the cost function
def cost_function(params, initial_poses, observed_positions):
    # Reshape the flattened parameters back into the original structure
    reshaped_params = params.reshape((len(initial_poses), -1))
    
    # Calculate the difference between predicted and observed marker positions
    residuals = np.concatenate([predicted_positions - observed_positions[i] for i, predicted_positions in enumerate(reshaped_params)])
    
    # Return the sum of squared residuals
    return np.sum(residuals ** 2)

# Main particle filter function with Aruco marker detection
def pose_estimation(cap, marker_locations):
    aruco_dict_type = ARUCO_DICT["DICT_6X6_250"]

    matrix_coefficients = np.load("./calibration_matrix.npy")
    distortion_coefficients = np.load("./distortion_coefficients.npy")

    img_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    img_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    while True:
        initial_poses = [0, 0]
        initial_poses_flat = np.concatenate(initial_poses)
        observed_positions = []
        
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if platform == "linux": # cv2 V4.5
            dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
            parameters = cv2.aruco.DetectorParameters_create()
            corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
        else:  # cv2 V4.7+
            dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(dictionary, parameters)

        corners, ids, rejected_img_points = detector.detectMarkers(gray)

        if len(corners) > 0:
            i = 0

            x = []
            y = []

            camera_poses = []

            marker_size = 25.41  # mm
            
            # Object points
            objp = np.array([[-marker_size / 2, marker_size / 2, 0],
                                        [marker_size / 2, marker_size / 2, 0],
                                        [marker_size / 2, -marker_size / 2, 0],
                                        [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)

            for corner in corners:

                mtx, roi = cv2.getOptimalNewCameraMatrix(matrix_coefficients, distortion_coefficients, (frame.shape[0], frame.shape[1]), 0.5, (frame.shape[0], frame.shape[1]))
                ret, rvec, tvec = cv2.solvePnP(objp, corner, mtx, None, False, cv2.SOLVEPNP_IPPE_SQUARE)
                # ret, rvec, tvec = cv2.solvePnP(objp, corner, matrix_coefficients, distortion_coefficients, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                rot_M = cv2.Rodrigues(rvec)[0]
                pos = -rot_M.T @ tvec

                print('World Position', marker_locations[ids[i][0]][0]-pos[0], marker_locations[ids[i][1]][0]-pos[1])
                x.append(marker_locations[ids[i][0]][0]-pos[0])
                y.append(marker_locations[ids[i][1]][0]-pos[1])
                
                camera_poses.append([marker_locations[ids[i][0]][0]-pos[0], marker_locations[ids[i][1]][0]-pos[1]])

                i += 1

            result = []
            if len(corners) > 2: 
                initial_poses = np.concatenate(camera_poses, axis=1)
                observed_positions = camera_poses

                initial_poses_flat = initial_poses.flatten()
                result = minimize(cost_function, initial_poses_flat, args=(observed_positions), method='L-BFGS-B')
            print(result)

            # Check Standard Deviation
            # print(f'Std - X:{np.std(x)}, Y:{np.std(y)}')

            # plt.scatter(x, y)
            # plt.show()

        # Retrieve refined camera poses
        refined_poses_flat = result.x
        refined_poses = refined_poses_flat.reshape((len(initial_poses), -1))

        print(refined_poses)
        
        cv2.imshow('Frame',frame)
        
        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Function to draw the estimated pose on the frame
def draw_estimated_pose(frame, pose):
    pose_int = tuple(map(int, pose))
    cv2.circle(frame, pose_int, 10, (0, 255, 0), -1)

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

# Example usage
if __name__ == "__main__":
    # Replace the following parameters with your actual camera calibration data
    cap = cv2.VideoCapture(0)

    marker_locations = manual_analyze_stitched()

    # Run the particle filter with Aruco marker detection
    pose_estimation(cap, marker_locations)
