FROM danieltobon43/pcl-docker-1-9-1:1.9.1

ENV USERNAME pcl

# ======== Copy dbscan project to docker container ========
WORKDIR /home/$USERNAME/project
COPY . .

# ======== Set executable permission ========
RUN sudo chown -R $USERNAME:$USERNAME pcd/
RUN sudo chown -R $USERNAME:$USERNAME build/

# ======== Compile dbscan project ========
RUN cd build/ && cmake ../ && make -j$(nproc)

ENV MESA_LOADER_DRIVER_OVERRIDE i965

# ======== Run binary file ========
ENTRYPOINT ["./build/dbscan"]

