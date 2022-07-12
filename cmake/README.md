# Generating Dependency Graphs with CMake
By using CMake you can automatically generate dependency graphs
(dot/graphviz) of the targets in your project. This works for
dependencies within the project, as well as dependencies to external
libraries.

![dbscan-dependencies](https://user-images.githubusercontent.com/35694200/178576867-4a78b269-8abd-46c3-912e-a01bb812713e.png)

## Usage
To use it, run cmake in the build tree:
```
cmake --graphviz=test.dot .
```

## Variables specific to the Graphviz support
The resulting graphs can be huge. The look and content of the generated graphs can be controlled using the file `CMakeGraphVizOptions.cmake`. This file is first searched in `CMAKE_BINARY_DIR`, and then in `CMAKE_SOURCE_DIR`. If found, the variables set in it are used to adjust options for the generated Graphviz files.

## Graphviz (.dot) file to .png
```
dot -Tpng test.dot -o foo.png
```

## Reference
- https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/Graphviz
- https://cmake.org/cmake/help/latest/module/CMakeGraphVizOptions.html