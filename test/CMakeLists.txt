project(tests LANGUAGES CXX)

include_directories(
    ${CMAKE_BINARY_DIR}/install/include
    ${CMAKE_SOURCE_DIR}/src
)

add_executable(tests tests.cpp)

target_include_directories(tests
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
)

target_link_directories(tests
    PRIVATE
        ${CMAKE_BINARY_DIR}/install/lib
)

target_link_libraries(tests
    PRIVATE
        gtest
        gtest_main
)

