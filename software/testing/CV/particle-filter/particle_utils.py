import cv2
import numpy as np
import matplotlib.pyplot as plt
from sys import platform

# Function to generate initial set of particles
def generate_particles(num_particles, img_width, img_height):
    particles = np.random.rand(num_particles, 3)
    particles[:, 0] *= img_width  # x-coordinate
    particles[:, 1] *= img_height  # y-coordinate
    particles[:, 2] = np.random.uniform(0, 2 * np.pi, num_particles)  # orientation
    return particles

# Function to update particles based on motion model (e.g., constant velocity)
def update_particles(particles, motion_model_params):
    particles[:, 0] += motion_model_params[0] * np.cos(particles[:, 2])  # update x
    particles[:, 1] += motion_model_params[0] * np.sin(particles[:, 2])  # update y
    particles[:, 2] += motion_model_params[1]  # update orientation
    return particles

# Function to compute particle weights based on observation model (e.g., using image features)
def compute_weights(particles, observation_model_params, observed_features):
    # Compute weights based on the difference between observed and predicted features
    predicted_features = np.mean(particles[:, :2], axis=0)  # Use the mean position of particles as the predicted feature
    weights = np.exp(-np.sum((observed_features - predicted_features)**2, axis=0) / (2 * observation_model_params[2]**2))
    return weights / np.sum(weights)

# Function to resample particles based on their weights
def resample_particles(particles, weights):
    num_particles = len(particles)
    indices = np.random.choice(num_particles, num_particles, p=weights/np.sum(weights))  # Normalize weights before resampling
    return particles[indices]

# Main particle filter function with Aruco marker detection
def particle_filter_with_aruco(cap, num_particles, motion_model_params, observation_model_params):
    aruco_dict_type = ARUCO_DICT["DICT_6X6_250"]

    matrix_coefficients = np.load("./calibration_matrix.npy")
    distortion_coefficients = np.load("./distortion_coefficients.npy")

    img_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    img_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    while True:
        
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

            x = []
            y = []

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

                print('World Posiion', pos[0], pos[1])
                x.append(pos[0])
                y.append(pos[1])
                # x.append(tvec[0])
                # y.append(tvec[1])

            # Check Standard Deviation
            print(f'Std - X:{np.std(x)}, Y:{np.std(y)}')

            plt.scatter(x, y)
            plt.show()
        
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
    camera_matrix = np.load("./calibration_matrix_2.npy")
    dist_coeff = np.load("./distortion_coefficients_2.npy")

    # Replace the following parameters with your actual values
    num_particles = 200
    motion_model_params = [1.0, 0.1]  # [velocity, angular velocity]
    observation_model_params = np.array([[1, 0], [0, 1], [0.1, 0.1]])  # [a, b, sigma]
    
    cap = cv2.VideoCapture(0)

    # Run the particle filter with Aruco marker detection
    particle_filter_with_aruco(cap, num_particles, motion_model_params, observation_model_params)
