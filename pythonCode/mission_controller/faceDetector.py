import cv2
import face_recognition
import time

class FaceDetection:
    def __init__(self, bounding_box=(0, 0, 0,0), face_id="Unknown", encoding=None):
        self.bounding_box = bounding_box  # (x, y, w, h)
        self.face_id = face_id            # "Friend_1", "Unknown", etc.
        self.encoding = encoding          # Face encoding (128-dimensional vector)

class TargetSelector:
    '''
    this class handles picking the right target to follow and shoot on
    '''
    def __init__(self):
        self.currentChosenTarget = FaceDetection()
    
    def pickAnEnemyDetection(self, faceDetections = []) :
        for faceDetection in faceDetections :
            if faceDetection.face_id == "Unknown" :
                self.currentChosenTarget = faceDetection
                return True
        return False


class FaceClassifier:
    def __init__(self):
        self.target_fps = 2
        self.interval = 1.0 / self.target_fps
        self.last_detection_time = time.time()
        self.faceDetected = False

        # Known faces and their encodings
        self.known_face_encodings = []
        self.known_face_names = []

        # Modes
        self.mode = "friendLearning"  # "friendLearning" or "friendsAndFoesDetection"

    def set_mode(self, mode):
        if mode not in ["friendLearning", "friendsAndFoesDetection"]:
            raise ValueError("Invalid mode. Use 'friendLearning' or 'friendsAndFoesDetection'.")
        self.mode = mode

    def process_frame(self, frame):
        # Resize frame for faster processing
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        
        # Detect face locations and encodings
        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

        detected_faces = []

        if self.mode == "friendLearning":
            for location, encoding in zip(face_locations, face_encodings):
                matches = face_recognition.compare_faces(self.known_face_encodings, encoding, tolerance=0.6)
                
                if not any(matches):
                    self.known_face_encodings.append(encoding)
                    face_id = "Friend_{}".format(len(self.known_face_names) + 1)
                    self.known_face_names.append(face_id)
                else:
                    match_index = matches.index(True)
                    face_id = self.known_face_names[match_index]
                
                top, right, bottom, left = location
                bounding_box = (left * 4, top * 4, (right - left) * 4, (bottom - top) * 4)

                detected_faces.append(FaceDetection(bounding_box, face_id, encoding))

        elif self.mode == "friendsAndFoesDetection":
            for location, encoding in zip(face_locations, face_encodings):
                matches = face_recognition.compare_faces(self.known_face_encodings, encoding, tolerance=0.6)
                face_id = "Unknown"
                if True in matches:
                    match_index = matches.index(True)
                    face_id = self.known_face_names[match_index]
                
                top, right, bottom, left = location
                bounding_box = (left * 4, top * 4, (right - left) * 4, (bottom - top) * 4)

                detected_faces.append(FaceDetection(bounding_box, face_id, encoding))

        return detected_faces


def main():
    # Initialize FaceClassifier
    face_classifier = FaceClassifier()

    # Open video stream
    video = cv2.VideoCapture(0)

    while True:
        ret, frame = video.read()
        if not ret:
            break

        # Process the frame
        detected_faces = face_classifier.process_frame(frame)

        # Draw the picture frame border based on the mode
        if face_classifier.mode == "friendLearning":
            border_color = (255, 255, 0)  # Cyan for friendLearning
        else:
            border_color = (0, 0, 255)  # Red for friendsAndFoesDetection

        frame = cv2.copyMakeBorder(frame, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=border_color)

        # Draw bounding boxes and labels for each detected face
        for face in detected_faces:
            x, y, w, h = face.bounding_box
            if face.face_id.startswith("Friend"):
                color = (255, 0, 0)  # Blue for friends
            else:
                color = (0, 0, 255)  # Red for unknowns

            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(frame, face.face_id, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Display the frame
        cv2.imshow('Face Detection', frame)

        # Keyboard controls
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break  # Quit
        elif key == ord('l'):
            face_classifier.set_mode("friendLearning")
        elif key == ord('d'):
            face_classifier.set_mode("friendsAndFoesDetection")

    # Release resources
    video.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
