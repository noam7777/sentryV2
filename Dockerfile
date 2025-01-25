# Use an official Python image with Python 3.9 as the base image
FROM python:3.9-slim

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    ffmpeg \
    libsm6 \
    libxext6 \
    libgl1-mesa-glx \
    v4l-utils \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install OpenCV for Python
RUN pip install --no-cache-dir opencv-python

# Create a working directory
WORKDIR /app

COPY pythonCode /app/

# Copy your Python script into the container (optional)
# Uncomment the line below if you have a script to include
# COPY your_script.py .

# Set the container to run bash by default
CMD ["bash"]



# how to run:
    
#run on host: "xhost +local:""

#docker run -it --rm --device=/dev/video0:/dev/video0 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix webcam-opencv

#to restrict the display to the local machine only
# when done, on the host machine run: "xhost -local:"


















# # Use a base image with Python
# FROM python:3.9-slim

# # Install system dependencies (done as root by default)
# RUN apt-get update && apt-get install -y \
#     build-essential \
#     cmake \
#     libopenblas-dev \
#     liblapack-dev \
#     libx11-dev \
#     libgtk-3-dev \
#     libboost-python-dev \
#     libboost-thread-dev \
#     python3-dev \
#     && rm -rf /var/lib/apt/lists/*

# # Create a non-root user and switch to it
# RUN useradd -m appuser
# USER appuser

# # Set the working directory for the non-root user
# WORKDIR /app

# # Add local bin to PATH
# ENV PATH="/home/appuser/.local/bin:${PATH}"

# # Install Python dependencies as the non-root user
# RUN pip install --no-cache-dir --upgrade pip
# RUN pip install --no-cache-dir face_recognition opencv-python

# # Copy only the necessary folders and files into the container
# COPY pythonCode /app/pythonCode

# # Define the command to run your Python script
# CMD ["python", "pythonCode/mission_controller/faceDetector.py"]

# # Run the container with access to the recording folder:
# # docker run --rm -it -v "$(pwd)/recordings:/app/recordings" --device=/dev/video0 sentry-project








