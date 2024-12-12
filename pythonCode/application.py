import cv2
import tkinter as tk
from tkinter import Label
from PIL import Image, ImageTk

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Camera Capture with OpenCV")
        self.root.geometry("800x600")

        # Create a Label widget for displaying the video
        self.video_label = Label(self.root)
        self.video_label.pack()

        # Open the camera
        self.cap = cv2.VideoCapture(0)  # Use camera index 0

        # Start updating the frames
        self.update_frame()

    def update_frame(self):
        # Capture frame-by-frame
        ret, frame = self.cap.read()
        if ret:
            # Process the frame (convert to grayscale for example)
            processed_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Convert the processed frame to RGB for Tkinter
            rgb_frame = cv2.cvtColor(processed_frame, cv2.COLOR_GRAY2RGB)

            # Convert the frame to a PIL Image and then to ImageTk
            img = Image.fromarray(rgb_frame)
            imgtk = ImageTk.PhotoImage(image=img)

            # Update the Label widget with the new frame
            self.video_label.imgtk = imgtk
            self.video_label.configure(image=imgtk)

        # Schedule the next frame update
        self.root.after(10, self.update_frame)

    def on_close(self):
        # Release the camera and destroy the window
        self.cap.release()
        self.root.destroy()

# Create and run the Tkinter application
if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)  # Handle window close
    root.mainloop()
