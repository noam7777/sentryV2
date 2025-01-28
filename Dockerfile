# Use an official Python image with Python 3.9 as the base image
FROM python:3.9-slim

# Install system dependencies
RUN apt-get update && apt-get install -y \
    ffmpeg \
    libsm6 \
    libxext6 \
    libgl1-mesa-glx \
    v4l-utils \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Create a new user with a home directory
RUN useradd -m appuser

# Set the working directory
WORKDIR /app

# Create a virtual environment as root (temporary)
RUN python -m venv /venv

# Change ownership of the working directory and the virtual environment
RUN chown -R appuser:appuser /app /venv

# Switch to the new user
USER appuser

# Activate the virtual environment and install dependencies
RUN /venv/bin/pip install --no-cache-dir --upgrade pip
RUN /venv/bin/pip install --no-cache-dir opencv-python

# Set the PATH to include the virtual environment's binaries
ENV PATH="/venv/bin:$PATH"

COPY pythonCode /app/pythonCode/

# Default command to run the container
CMD ["bash"]



# # Use an official Python image with Python 3.9 as the base image
# FROM python:3.9-slim

# # Install system dependencies
# RUN apt-get update && apt-get install -y \
#     ffmpeg \
#     libsm6 \
#     libxext6 \
#     libgl1-mesa-glx \
#     v4l-utils \
#     && apt-get clean && rm -rf /var/lib/apt/lists/*

# # Create a new user with a home directory
# RUN useradd -m appuser

# # Set the working directory
# WORKDIR /app

# # Change ownership of the working directory to the new user
# RUN chown -R appuser:appuser /app /venv

# # Switch to the new user
# USER appuser

# # Create a virtual environment (as the new user)
# RUN python -m venv /venv

# # Activate the virtual environment and install dependencies
# RUN /venv/bin/pip install --no-cache-dir --upgrade pip
# RUN /venv/bin/pip install --no-cache-dir opencv-python

# # Set the PATH to include the virtual environment's binaries
# ENV PATH="/venv/bin:$PATH"

# # Copy your script into the container
# COPY test_webcam.py /app/

# # Default command to run the container
# CMD ["bash"]


# # Use an official Python image with Python 3.9 as the base image
# FROM python:3.9-slim

# # Install system dependencies
# RUN apt-get update && apt-get install -y \
#     ffmpeg \
#     libsm6 \
#     libxext6 \
#     libgl1-mesa-glx \
#     v4l-utils \
#     && apt-get clean && rm -rf /var/lib/apt/lists/*

# # Set the working directory
# WORKDIR /app

# # Create a virtual environment
# RUN python -m venv /venv

# # Activate the virtual environment and install dependencies
# RUN /venv/bin/pip install --no-cache-dir --upgrade pip
# RUN /venv/bin/pip install --no-cache-dir opencv-python

# # Set the PATH to include the virtual environment's binaries
# ENV PATH="/venv/bin:$PATH"

# # Copy your script into the container
# COPY test_webcam.py /app/

# # Default command to run the container
# CMD ["bash"]




# how to run:
    
#run on host: "xhost +local:"

#docker run -it --rm --device=/dev/video0:/dev/video0 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix webcam-opencv
#
# docker run -it --rm --device=/dev/video0:/dev/video0 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/noam/Documents/personalProjects/sentryV2/venv-sentry:/venv-sentry -v pythonCode:/app/pythonCode -w /app webcam-opencv
# python pythonCode/test_scripts/test_web_cam.py
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








