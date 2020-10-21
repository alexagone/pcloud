# Domain translation training

    Implementation of a point cloud convex hull estimation algorithm based ont quickhull.

    The implementation is provided in a single header file in src/PointCloud.hpp

    A simple usage example is provided in src/main.cpp


# Build

    You can build nailed it using the following:

        mkdir build
        cd build
        cmake ..
        make

    This will compile the application to the following location

        build/src/nailedit


# Tests (optional)

    (Only manual cloning of googletest is supported for now...)

    Optionally, tests can be compiled by cloning [googletest](https://github.com/google/googletest.git) repository in third/ folder.


        git clone https://github.com/google/googletest.git third/googletest

    And runing the build sequence again:

        cd build
        cmake ..
        make
        

    Tests can be run using:

        ./build/test/nailedit_tests


# Runing the application

   Demo application can be run after a successful build using:

        ./build/src/nailedit
