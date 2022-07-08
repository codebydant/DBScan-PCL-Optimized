# Allow X server connection
xhost +local:root
docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume=`pwd`:/home/pcluser/project \
    danieltobon43/pcl-docker:1.12.1-alpine3.15-All-dev
# Disallow X server connection
xhost -local:root