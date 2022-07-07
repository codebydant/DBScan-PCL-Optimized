# Build stage
FROM danieltobon43/pcl-docker:1.12.1-alpine3.15-All AS runtime

# ======== Compile dbscan project ========
COPY build/install/ /usr

# WORKDIR /tmp

ENTRYPOINT ["/bin/app"]
# # Runtime
# FROM danieltobon43/pcl-docker:1.12.1-alpine3.15-All as runtime
# # ENV MESA_LOADER_DRIVER_OVERRIDE i965
# RUN apk upgrade --no-cache && \
# 	apk add --no-cache libgcc libstdc++

# COPY --from=build_dbscan /tmp/install /usr

# # ======== Run binary file ========
# ENTRYPOINT ["app"]

# vtk 9.0 
# error: vtkContextDevice2D.cxx:32 WARN| Error: no override found for 'vtkContextDevice2D
# https://discourse.vtk.org/t/two-questions-about-new-module-system/2864/16