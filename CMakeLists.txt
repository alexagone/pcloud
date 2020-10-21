cmake_minimum_required(VERSION 3.9 FATAL_ERROR)

project(nailed_it VERSION 0.0.0.1 LANGUAGES C CXX)

#base compilation flags
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-unused-parameter -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")

add_subdirectory(third)
add_subdirectory(src)
add_subdirectory(test)
