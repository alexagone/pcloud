project(nailedit_tests LANGUAGES CXX)

include_directories(
    ${CMAKE_BINARY_DIR}/install/include
    ${CMAKE_SOURCE_DIR}/src
)

add_executable(nailedit_tests tests.cpp)

target_link_directories(nailedit_tests
    PRIVATE
        ${CMAKE_BINARY_DIR}/install/lib
)

target_link_libraries(nailedit_tests
    PRIVATE
        gtest
        gtest_main
)

