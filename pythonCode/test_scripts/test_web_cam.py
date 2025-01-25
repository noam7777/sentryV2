import cv2

# Open the webcam
cap = cv2.VideoCapture(0)  # 0 corresponds to /dev/video0

if not cap.isOpened():
    print("Error: Cannot access webcam.")
    exit()

print("Press 'q' to exit.")
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to read frame.")
        break

    # Display the video stream
    cv2.imshow("Webcam Video Stream", frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()