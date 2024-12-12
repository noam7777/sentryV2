import cv2
import numpy as np
import time
from SentryController import Controller
from sentryTracker import Tracker


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

# Variables to store face bounding box, features, and weights
faces = []

# Initialize mask to be the same size as the frame
ret, frame = cap.read()
if not ret:
    print("Error: Unable to capture the first frame.")
    cap.release()
    cv2.destroyAllWindows()
    exit()

# Create a mask image with the same shape as the frame
mask = np.zeros_like(frame)
faceDetected = False

sentryController = Controller()
tracker = Tracker()

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
            faceDetected = True
            # Get the bounding box of the first detected face
            x, y, w, h = faces[0]

            tracker.findNewFeatures(gray, faces[0])
            tracker.updateWeights(faces[0])

        else:
            faceDetected = False


    # Only proceed with optical flow if there are points to track
    if tracker.p0 is not None and len(tracker.p0) > 0:
        tracker.trackFeaturesAndDrawMotion(gray, frame, mask)

    # Overlay mask with motion vectors onto frame
    img = cv2.add(frame, mask)

    # Draw the bounding box around the face
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)

# lock on target mode
    if tracker.p0 is not None and len(tracker.weights) > 0:
        avg_position_normalized = calculate_and_mark_average_position(tracker.p0, tracker.weights, img)
        if avg_position_normalized:
            print(f"Average position: {avg_position_normalized}")
            sentryController.sentry_pid(avg_position_normalized, faceDetected)


    # Display the result
    cv2.imshow('Face Detection with Optical Flow and Weights', img)

    # Update the old frame and old points
    old_gray = gray.copy()


    # Exit on ESC key
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
