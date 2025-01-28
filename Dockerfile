# Use an official Python image with Python 3.9 as the base image
FROM python:3.9-slim

# Install system dependencies, including tk and tcl for tkinter
RUN apt-get update && apt-get install -y \
    ffmpeg \
    libsm6 \
    libxext6 \
    libgl1-mesa-glx \
    v4l-utils \
    tk \
    tcl \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
 
# update pip
RUN python3.9 -m pip install --upgrade pip

# Set the working directory
WORKDIR /app

# Set the PATH to include the virtual environment's binaries
ENV PATH="/home/appuser/.local/bin:$PATH"

# Activate the virtual environment and install dependencies
RUN pip install --no-cache-dir opencv-python
RUN pip install --no-cache-dir tk
RUN pip install --no-cache-dir pillow
RUN pip install --no-cache-dir pyserial



#copy code to the container
COPY pythonCode /app/pythonCode/

# Default command to run the container
CMD ["bash"]



# how to run:
    
#run on host: "xhost +local:""

#docker run -it --rm --device=/dev/video0:/dev/video0 --device=/dev/ttyUSB0:/dev/ttyUSB0 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix webcam-opencv
# then run "python3 pythonCode/mission_controller/applicationArduinoGui.py"
#to restrict the display to the local machine only
# when done, on the host machine run: "xhost -local:"
