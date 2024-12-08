import cv2
import numpy as np
import time
import serial

kP_azimuth = 80
kP_elevation = 120

# Configure the serial connection
arduino_port = '/dev/ttyUSB0'  # Replace with the correct port for your Arduino
baud_rate = 9600
serial_conn = serial.Serial(arduino_port, baud_rate)
time.sleep(2)  # Allow the connection to initialize

targetErrorToShoot = 0.1
targetCenteredCountToShoot = 5

def send_command(elevation_velocity, azimuth_velocity, should_shoot):
    """
    Sends a structured command to the Arduino.
    :param elevation_velocity: Elevation angular velocity in DecidegreesPerSec (integer).
    :param azimuth_velocity: Azimuth angular velocity in DecidegreesPerSec (integer).
    :param should_shoot: 1 to shoot, 0 otherwise (integer).
    """
    command = f"{elevation_velocity},{azimuth_velocity},{should_shoot}\n"
    serial_conn.write(command.encode())
    print(f"Sent command: {command.strip()}")

def sentry_pid(avg_position):
    if not hasattr(sentry_pid, "targetCenteredCounter"):
        sentry_pid.targetCenteredCounter = 0  # Initialize static variable    targetPosX, targetPosY = avg_position

    targetPosX, targetPosY = avg_position
    errorX = -(targetPosX - 0.5)
    errorY = (targetPosY - 0.5)
    if (((errorX ** 2 + errorY ** 2) ** 0.5) < targetErrorToShoot):
        sentry_pid.targetCenteredCounter += 1
    else:
        sentry_pid.targetCenteredCounter = 0

    if sentry_pid.targetCenteredCounter > targetCenteredCountToShoot :
        shouldShoot = 1
    else:
        shouldShoot = 0

    cmdAzimuth = int(errorX * kP_azimuth)
    cmdElevation = int(errorY * kP_elevation)
    send_command(cmdElevation, cmdAzimuth, shouldShoot)  # Move servos at 15°/sec and initiate shooting



def calculate_and_mark_average_position(features, weights, image):
    """
    Calculate the weighted average position of features and mark it on the image.

    Args:
        features (numpy.ndarray): Array of shape (N, 1, 2) containing the positions of the features.
        weights (list): List of weights corresponding to the features.
        image (numpy.ndarray): The image on which to mark the average position.

    Returns:
        tuple: The average position normalized to range [0, 1] in both axes (x_avg_norm, y_avg_norm).
    """
    if features is None or len(features) == 0 or len(weights) == 0:
        return None

    # Ensure weights and features have matching lengths
    if len(features) != len(weights):
        raise ValueError("The number of features and weights must be the same.")

    # Normalize weights to ensure they sum to 1
    weights = np.array(weights, dtype=np.float32)
    if np.sum(weights) == 0:
        return None
    weights /= np.sum(weights)

    # Calculate weighted average position
    weighted_sum = np.sum(features[:, 0, :] * weights[:, None], axis=0)
    x_avg, y_avg = weighted_sum[0], weighted_sum[1]

    # Normalize the average position to range [0, 1]
    height, width = image.shape[:2]
    x_avg_norm = x_avg / width
    y_avg_norm = y_avg / height

    # Mark the average position on the image
    marker_color = (0, 255, 0)  # Green color for the marker
    marker_size = 10
    cv2.drawMarker(image, (int(x_avg), int(y_avg)), marker_color, markerType=cv2.MARKER_CROSS, markerSize=marker_size, thickness=2)

    return x_avg_norm, y_avg_norm

# Load the Haar cascade for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Set up video capture
cap = cv2.VideoCapture(0)

# Face detection frame rate control
target_fps = 5

interval = 1.0 / target_fps
last_detection_time = time.time()

# Parameters for Shi-Tomasi corner detection within the face bounding box
feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)

# Parameters for Lucas-Kanade optical flow
lk_params = dict(winSize=(15, 15), maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# Random colors for motion vectors
color = np.random.randint(0, 255, (100, 3))

# Variables to store face bounding box, features, and weights
faces = []
p0 = None
weights = []  # List to hold weights for each feature

# Parameters for weight handling
weight_increment = 0.1
weight_decrement = 0.5
weight_threshold = 0.1  # Threshold below which features are deleted if outside the bounding box

# Initialize mask to be the same size as the frame
ret, frame = cap.read()
if not ret:
    print("Error: Unable to capture the first frame.")
    cap.release()
    cv2.destroyAllWindows()
    exit()

# Create a mask image with the same shape as the frame
mask = np.zeros_like(frame)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        break

    # Reset mask to match the current frame's dimensions
    mask = np.zeros_like(frame)

    # Convert to grayscale for processing
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Perform face detection at the specified interval
    current_time = time.time()
    if (current_time - last_detection_time) >= interval:
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
        last_detection_time = current_time

        if len(faces) > 0:
            # Get the bounding box of the first detected face
            x, y, w, h = faces[0]

            # If the number of tracked points is less than 10, find new points in the bounding box
            if p0 is None or len(p0) < 10:
                roi_gray = gray[y:y+h, x:x+w]
                new_features = cv2.goodFeaturesToTrack(roi_gray, mask=None, **feature_params)

                # Adjust coordinates of the new features to the full image
                if new_features is not None:
                    new_features[:, 0, 0] += x
                    new_features[:, 0, 1] += y
                    new_features = np.float32(new_features)

                    if p0 is None:
                        p0 = new_features
                        weights = [0] * len(p0)
                    else:
                        # Append new features and initialize their weights to 0
                        p0 = np.vstack((p0, new_features))
                        weights.extend([0] * len(new_features))

            # Increment/decrement weights and filter points based on bounding box and threshold
            if p0 is not None and len(p0) > 0:
                filtered_p0 = []
                filtered_weights = []
                
                for i, point in enumerate(p0):
                    a, b = point.ravel()
                    
                    # Check if the point is within the bounding box
                    if x <= a <= x+w and y <= b <= y+h:
                        # Increment weight if inside bounding box
                        weights[i] = min(1.0, weights[i] + weight_increment)
                    else:
                        # Decrement weight if outside the bounding box
                        weights[i] = max(0, weights[i] - weight_decrement)

                    # Keep the point if it's inside the bounding box or if it has a weight above the threshold
                    if weights[i] > weight_threshold or (x <= a <= x+w and y <= b <= y+h):
                        filtered_p0.append(point)
                        filtered_weights.append(weights[i])
                
                # Update p0 and weights to include only retained points
                p0 = np.array(filtered_p0).reshape(-1, 1, 2)
                weights = filtered_weights

                # If all points are filtered out, reset p0 and weights
                if len(p0) == 0:
                    p0 = None
                    weights = []

    # Only proceed with optical flow if there are points to track
    if p0 is not None and len(p0) > 0:
        # Calculate optical flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray if 'old_gray' in locals() else gray, gray, p0, None, **lk_params)
        
        # Select good points and keep weights for points that were successfully tracked
        if p1 is not None and st is not None:
            good_new = p1[st == 1]
            good_old = p0[st == 1]
            weights = [weights[i] for i, status in enumerate(st) if status == 1]

            # Draw the motion vectors with weights
            for i, (new, old, weight) in enumerate(zip(good_new, good_old, weights)):
                a, b = new.ravel()
                c, d = old.ravel()
                mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)

                # Scale radius based on weight
                radius = int(5 + 10 * weight)
                frame = cv2.circle(frame, (int(a), int(b)), radius, color[i].tolist(), -1)
            
            # Update p0 for the next iteration
            p0 = good_new.reshape(-1, 1, 2)

    # Overlay mask with motion vectors onto frame
    img = cv2.add(frame, mask)

    # Draw the bounding box around the face
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)


    if p0 is not None and len(weights) > 0:
        avg_position_normalized = calculate_and_mark_average_position(p0, weights, img)
        if avg_position_normalized:
            print(f"Average position: {avg_position_normalized}")
            sentry_pid(avg_position_normalized)


    # Display the result
    cv2.imshow('Face Detection with Optical Flow and Weights', img)

    # Update the old frame and old points
    old_gray = gray.copy()


    # Exit on ESC key
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
