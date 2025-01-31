import pickle
import os

class FaceEncodingManager:
    """Manages saving and loading face encodings and names."""
    
    def __init__(self, file_path="/app/data/face_data.pkl"):
        self.file_path = file_path
        self.known_face_encodings, self.known_face_names = self.load_data()

    def save_data(self):
        """Save the known face encodings and names to a file."""
        try:
            with open(self.file_path, "wb") as f:
                pickle.dump((self.known_face_encodings, self.known_face_names), f)
            print(f"Saved {len(self.known_face_encodings)} face encodings and names to {self.file_path}")
        except Exception as e:
            print(f"Error saving face data: {e}")

    def load_data(self):
        """Load the known face encodings and names from a file."""
        if os.path.exists(self.file_path):
            try:
                with open(self.file_path, "rb") as f:
                    encodings, names = pickle.load(f)
                print(f"Loaded {len(encodings)} face encodings and names from {self.file_path}")
                return encodings, names
            except Exception as e:
                print(f"Error loading face data: {e}")
                return [], []
        return [], []

    def add_face(self, encoding, name):
        """Add a new face encoding and name, then save."""
        self.known_face_encodings.append(encoding)
        self.known_face_names.append(name)

    def remove_face(self, name):
        """Remove a face encoding and name by name."""
        if name in self.known_face_names:
            index = self.known_face_names.index(name)
            del self.known_face_encodings[index]
            del self.known_face_names[index]
            print(f"Removed face: {name}")
        else:
            print(f"Face '{name}' not found.")

    def clear_data(self):
        """Clear all saved encodings and names."""
        self.known_face_encodings = []
        self.known_face_names = []
        self.save_data()
