import cv2
import numpy as np

class Tracker:
    def __init__(self):
        self.p0 = None
        self.weights = []  # List to hold weights for each feature
        self.old_gray = None  # Previous grayscale frame for optical flow


        # Parameters for weight handling
        self.weight_increment = 0.1
        self.weight_decrement = 0.5
        self.weight_threshold = 0.1  # Threshold below which features are deleted if outside the bounding box

        # Parameters for Shi-Tomasi corner detection within the face bounding box
        self.feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)

        # Parameters for Lucas-Kanade optical flow
        self.lk_params = dict(winSize=(15, 15), maxLevel=2,
                        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        # Random colors for motion vectors
        self.color = np.random.randint(0, 255, (100, 3))

    def findNewFeatures(self, gray, bbox,):
        # Get the bounding box  of the detected face
        x, y, w, h = bbox
        # If the number of tracked points is less than 10, find new points in the bounding box
        if self.p0 is None or len(self.p0) < 10:
            roi_gray = gray[y:y+h, x:x+w]
            new_features = cv2.goodFeaturesToTrack(roi_gray, mask=None, **self.feature_params)

            # Adjust coordinates of the new features to the full image
            if new_features is not None:
                new_features[:, 0, 0] += x
                new_features[:, 0, 1] += y
                new_features = np.float32(new_features)

                if self.p0 is None:
                    self.p0 = new_features
                    self.weights = [0] * len(self.p0)
                else:
                    # Append new features and initialize their weights to 0
                    self.p0 = np.vstack((self.p0, new_features))
                    self.weights.extend([0] * len(new_features))

    def updateWeights(self, bbox):
        x, y, w, h = bbox

        # Increment/decrement weights and filter points based on bounding box and threshold
        if self.p0 is not None and len(self.p0) > 0:
            filtered_p0 = []
            filtered_weights = []
            
            for i, point in enumerate(self.p0):
                a, b = point.ravel()
                
                # Check if the point is within the bounding box
                if x <= a <= x+w and y <= b <= y+h:
                    # Increment weight if inside bounding box
                    self.weights[i] = min(1.0, self.weights[i] + self.weight_increment)
                else:
                    # Decrement weight if outside the bounding box
                    self.weights[i] = max(0, self.weights[i] - self.weight_decrement)

                # Keep the point if it's inside the bounding box or if it has a weight above the threshold
                if self.weights[i] > self.weight_threshold or (x <= a <= x+w and y <= b <= y+h):
                    filtered_p0.append(point)
                    filtered_weights.append(self.weights[i])
            
            # Update self.p0 and weights to include only retained points
            self.p0 = np.array(filtered_p0).reshape(-1, 1, 2)
            self.weights = filtered_weights

            # If all points are filtered out, reset self.p0 and weights
            if len(self.p0) == 0:
                self.p0 = None
                self.weights = []

    def trackFeaturesAndDrawMotion(self, gray, frame, mask):
        if self.old_gray is None:
            self.old_gray = gray.copy()
            return

        # Calculate optical flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, gray, self.p0, None, **self.lk_params)

        if p1 is not None and st is not None:
            good_new = p1[st == 1]
            good_old = self.p0[st == 1]
            self.weights = [self.weights[i] for i, status in enumerate(st) if status == 1]

            for i, (new, old, weight) in enumerate(zip(good_new, good_old, self.weights)):
                a, b = new.ravel()
                c, d = old.ravel()
                mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), self.color[i].tolist(), 2)

                radius = int(5 + 10 * weight)
                frame = cv2.circle(frame, (int(a), int(b)), radius, self.color[i].tolist(), -1)

            self.p0 = good_new.reshape(-1, 1, 2)

        self.old_gray = gray.copy()  # Update old frame for the next iteration

    def calculate_and_mark_average_position(self, image):
        """
        Calculate the weighted average position of features and mark it on the image.

        Args:
            features (numpy.ndarray): Array of shape (N, 1, 2) containing the positions of the features.
            weights (list): List of weights corresponding to the features.
            image (numpy.ndarray): The image on which to mark the average position.

        Returns:
            tuple: The average position normalized to range [0, 1] in both axes (x_avg_norm, y_avg_norm).
        """
        if self.p0 is None or len(self.p0) == 0 or len(self.weights) == 0:
            return None

        # Ensure weights and self.features have matching lengths
        if len(self.p0) != len(self.weights):
            raise ValueError("The number of features and weights must be the same.")

        # Normalize weights to ensure they sum to 1
        weightsNpArray = np.array(self.weights, dtype=np.float32)
        if np.sum(self.weights) == 0:
            return None
        weightsNpArray /= np.sum(weightsNpArray)

        # Calculate weighted average position
        weighted_sum = np.sum(self.p0[:, 0, :] * weightsNpArray[:, None], axis=0)
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
