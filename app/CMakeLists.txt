add_executable(pcloud main.cpp)

target_include_directories(pcloud
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
)
