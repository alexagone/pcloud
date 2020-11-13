COPYRIGHT NOTICE

Copyright (C) 2004-2020

Author: Alexandre Savard

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

# Domain translation training

    Implementation of a point cloud convex hull estimation algorithm based ont quickhull.

    The implementation is provided in a single header file in src/PointCloud.hpp

    A simple usage example is provided in src/main.cpp


# Build

    You can build pcloud it using the following:

        mkdir build
        cd build
        cmake ..
        make

    This will compile the application to the following location

        build/app/pcloud

    To compile in Release mode

        cmake -DCMAKE_BUILD_TYPE=Release ..

    To compile in Debug mode

        cmake -DCMKAE_BUILD_TYPE=Debug ..


# Tests (optional)

    (Only manual cloning of googletest is supported for now...)

    Optionally, tests can be compiled by cloning [googletest](https://github.com/google/googletest.git) repository in third/ folder.


        git clone https://github.com/google/googletest.git third/googletest

    And runing the build sequence again:

        cd build
        cmake ..
        make
        

    Tests can be run using:

        ./build/test/tests


# Runing the application

    Demo application can be run after a successful build using:

        ./build/app/pcloud

    This will generate two files that contain respectively the point cloud coordinates and convex hull estimated by the algorithm.

        /tmp/pcloud_cloud.csv
        /tmp/pcloud_hull.csv


# Point cloud visualization

    The following minimalistic python script is provided to visualize the generated point cloud and estimated convex hull.

        ./python/plot_cloud.py

# Packaging

    Simply:

        git archive HEAD --format=tar.gz --prefix=pcloud/ > pcloud.tar.gz
