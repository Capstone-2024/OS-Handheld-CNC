import cv2
import numpy as np

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
    predicted_features = observation_model_params.dot(particles[:, :2].T)
    weights = np.exp(-np.sum((observed_features - predicted_features)**2, axis=0) / (2 * observation_model_params[2]**2))
    return weights / np.sum(weights)

# Function to resample particles based on their weights
def resample_particles(particles, weights):
    indices = np.random.choice(len(particles), len(particles), p=weights)
    return particles[indices]

# Function to detect Aruco markers and their poses in the image
def detect_markers(img, dictionary, parameters, camera_matrix, dist_coeff):
    corners, ids, _ = cv2.aruco.detectMarkers(img, dictionary, parameters=parameters)
    if ids is not None:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 1.0, camera_matrix, dist_coeff)
        return ids, rvecs, tvecs
    else:
        return None, None, None

# Main particle filter function with Aruco marker detection
def particle_filter_with_aruco(cap, num_particles, motion_model_params, observation_model_params):
    img_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    img_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    particles = generate_particles(num_particles, img_width, img_height)

    while True:
        
        ret, frame = cap.read()
        if not ret:
            break

        # Detect Aruco markers and estimate poses
        ids, rvecs, tvecs = detect_markers(frame, aruco_dict, aruco_params, camera_matrix, dist_coeff)

        if ids is not None:
            observed_features = tvecs[0][0]  # Using the position of the first detected marker as the observed feature

            # Update particles based on motion model
            particles = update_particles(particles, motion_model_params)

            # Compute weights based on observation model
            weights = compute_weights(particles, observation_model_params, observed_features)

            # Resample particles
            particles = resample_particles(particles, weights)

            # Compute final estimated pose based on particle distribution
            estimated_pose = np.mean(particles, axis=0)[:2]

            # Draw the estimated pose on the frame
            draw_estimated_pose(frame, estimated_pose)

            # Display the frame
            cv2.imshow("Particle Filter with Aruco", frame)

            # Exit on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()

# Function to draw the estimated pose on the frame
def draw_estimated_pose(frame, pose):
    pose_int = tuple(map(int, pose))
    cv2.circle(frame, pose_int, 10, (0, 255, 0), -1)

# Replace the following parameters with your actual camera calibration data
camera_matrix = np.load("./calibration_matrix_2.npy")
dist_coeff = np.load("./distortion_coefficients_2.npy")

# Replace the following line with the path to your Aruco marker dictionary file
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters_create()

# Example usage
if __name__ == "__main__":
    # Replace the following parameters with your actual camera index or video file path
    camera_index = 0
    cap = cv2.VideoCapture(camera_index)

    # Replace the following parameters with your actual values
    num_particles = 200
    motion_model_params = [1.0, 0.1]  # [velocity, angular velocity]
    observation_model_params = np.array([[1, 0], [0, 1], [0.1, 0.1]])  # [a, b, sigma]

    # Run the particle filter with Aruco marker detection
    particle_filter_with_aruco(cap, num_particles, motion_model_params, observation_model_params)
