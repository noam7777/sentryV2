import cv2
import numpy as np
import time
from SentryController import Controller
from sentryTracker import Tracker
from faceDetector import FaceDetector
import threading


class LockAndShootController:
    def __init__(self, serial_conn):
        self.faceDetector = FaceDetector()
        self.sentryController = Controller(serial_conn)
        self.tracker = Tracker()
        self.shouldSendCommandsToRobot = False
        self.running = True

        # Set up video capture
        self.cap = cv2.VideoCapture(0)

        # Initialize mask to be the same size as the frame
        ret, frame = self.cap.read()
        if not ret:
            print("Error: Unable to capture the first frame.")
            self.cap.release()
            cv2.destroyAllWindows()
            exit()

        self.mask = np.zeros_like(frame)

    def start_auto_control_thread(self):
        thread = threading.Thread(target=self.auto_turret_pid_loop, daemon=True)
        thread.start()

    def auto_turret_pid_loop(self):
        while self.running:
            # Capture frame-by-frame
            ret, frame = self.cap.read()
            if not ret or not self.running:
                break

            # Reset mask
            self.mask = np.zeros_like(frame)

            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Perform face detection
            current_time = time.time()
            if (current_time - self.faceDetector.last_detection_time) >= self.faceDetector.interval:
                faces = self.faceDetector.detectTarget(gray)
                self.faceDetector.last_detection_time = current_time

                if len(faces) > 0:
                    self.faceDetector.faceDetected = True
                    selectedTarget = faces[0]
                    self.tracker.findNewFeatures(gray, selectedTarget)
                    self.tracker.updateWeights(selectedTarget)
                else:
                    self.faceDetector.faceDetected = False

            # Process optical flow
            if self.tracker.p0 is not None and len(self.tracker.p0) > 0:
                self.tracker.trackFeaturesAndDrawMotion(gray, frame, self.mask)

            # Overlay mask
            img = cv2.add(frame, self.mask)

            # Call a callback to update GUI (instead of cv2.imshow)
            self.update_gui_callback(img)

        self.cap.release()

    def update_gui_callback(self, frame):
        """
        Callback to update the Tkinter GUI with the latest frame.
        """
        # Convert OpenCV frame (BGR) to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Handle the conversion to Tkinter-compatible image here (e.g., using PIL or Canvas widget)
        pass

    def stop(self):
        self.running = False

