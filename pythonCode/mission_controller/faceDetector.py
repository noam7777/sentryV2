import cv2
import time
class FaceDetector:
    def __init__(self):
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.target_fps = 5
        self.interval = 1.0 / self.target_fps
        self.last_detection_time = time.time()
        self.facesBbox = []
        self.faceDetected = False

    def detectTarget(self, gray) :
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
        return faces