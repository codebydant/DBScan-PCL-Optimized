# xhost -local:root
# Allow X server connection
xhost +local:root
docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume=/home/danieltc/Downloads/models:/tmp/output \
    --entrypoint=/bin/sh \
    ghcr.io/danieltobon43/dbscan-octrees:goodapp
    # --entrypoint=/bin/sh \
    # --volume=`pwd`:/tmp/output \
        # --volume=/home/danieltc/Downloads/models:/tmp/output \
    
    # --entrypoint=/bin/sh \
    # --volume=`pwd`:/tmp \
    
    # ghcr.io/danieltobon43/dbscan-octrees:latest
# Disallow X server connection
xhost -local:root