echo "running container"
xhost +local:
docker run -it --rm \
    --device=/dev/video0:/dev/video0 \
    --device=/dev/ttyUSB0:/dev/ttyUSB0 \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd)/pythonCode:/app/pythonCode \
    -v $(pwd)/data:/app/data \
    -w /app \
    webcam-opencv
xhost -local:
echo "container is closed"