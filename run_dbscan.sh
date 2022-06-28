# Allow X server connection
xhost +local:root
docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume=/home/danieltc/Downloads/DBScan-PCL-Optimized:/tmp \
    --entrypoint /bin/sh \
    ghcr.io/danieltobon43/dbscan-pcl-optimized:test-github-packages
# Disallow X server connection
xhost -local:root