# vtk 9.0 
# error: vtkContextDevice2D.cxx:32 WARN| Error: no override found for 'vtkContextDevice2D
# https://discourse.vtk.org/t/two-questions-about-new-module-system/2864/16

# Build stage
FROM danieltobon43/pcl-docker:1.12.1-alpine3.15 as build_dbscan
 
# ======== Compile dbscan project ========
RUN apk --no-cache add cmake make g++

WORKDIR /usr/src

COPY app/ /usr/src

RUN mkdir -p /usr/src/install

RUN cmake -DCMAKE_INSTALL_PREFIX=/usr/src/install \
		  -S . -Bbuild && make -C /usr/src/build -Wno-cpp -j$(nproc) && make -C /usr/src/build install

# Runtime
FROM danieltobon43/pcl-docker:1.12.1-alpine3.15 as runtime
ENV MESA_LOADER_DRIVER_OVERRIDE i965
RUN apk --no-cache add mesa-dri-swrast

COPY --from=build_dbscan /usr/src/install/bin /usr/bin

# ======== Run binary file ========
ENTRYPOINT ["dbscan"]

