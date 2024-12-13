import cv2
import numpy as np
import time
from SentryController import Controller
from sentryTracker import Tracker




class FaceDetector:
    def __init__(self):
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.target_fps = 5
        self.interval = 1.0 / self.target_fps
        self.last_detection_time = time.time()
        self.facesBbox = []
        self.faceDetected = False

    def detectTarget(self) :
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
        return faces

faceDetector = FaceDetector()

# Set up video capture
cap = cv2.VideoCapture(0)

# Initialize mask to be the same size as the frame
ret, frame = cap.read()
if not ret:
    print("Error: Unable to capture the first frame.")
    cap.release()
    cv2.destroyAllWindows()
    exit()

# Create a mask image with the same shape as the frame
mask = np.zeros_like(frame)

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
    
    if (current_time - faceDetector.last_detection_time) >= faceDetector.interval:
        faces = faceDetector.detectTarget()
        faceDetector.last_detection_time = current_time

        if len(faces) > 0:
            faceDetector.faceDetected = True
            selectedTarget = faces[0]

            # Get the bounding box of the first detected face
            x, y, w, h = selectedTarget

            tracker.findNewFeatures(gray, selectedTarget)
            tracker.updateWeights(selectedTarget)

        else:
            faceDetector.faceDetected = False


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
        avg_position_normalized = tracker.calculate_and_mark_average_position(img)
        if avg_position_normalized:
            print(f"Average position: {avg_position_normalized}")
            sentryController.sentry_pid(avg_position_normalized, faceDetector.faceDetected)


    # Display the result
    cv2.imshow('Face Detection with Optical Flow and Weights', img)

    # Update the old frame and old points
    old_gray = gray.copy()

    arduino_data = sentryController.read_data()
    if arduino_data:
        print(f"Arduino says: {arduino_data}")

    # Exit on ESC key
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
