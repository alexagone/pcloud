cmake_minimum_required(VERSION 3.9 FATAL_ERROR)

project(pcloud VERSION 0.0.3.0 LANGUAGES C CXX)

#base compilation flags
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Wextra -Wno-unused-parameter -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")

if(EXISTS ${CMAKE_SOURCE_DIR}/third/googletest)
    add_subdirectory(third)
    add_subdirectory(test)
endif()

add_subdirectory(app)
