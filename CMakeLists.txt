cmake_minimum_required(VERSION 3.20)

project(register)


find_package(OpenCV)
find_package(Eigen3)
find_package(Boost REQUIRED COMPONENTS filesystem program_options)

include_directories(
    ${PROJECT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_library(calib SHARED
    ./src/calib.cc
    ./src/calib.h
    ./src/utility.cc
    ./src/utility.h
)

target_link_libraries(calib
    ${OpenCV_LIBS}
)

# example
add_executable(rgbd_calib ${PROJECT_SOURCE_DIR}/rgbd_calib.cc)
target_link_libraries(rgbd_calib
    calib
    ${Boost_LIBRARIES}
)
add_executable(rgbd_align ${PROJECT_SOURCE_DIR}/rgbd_align.cc)
target_link_libraries(rgbd_align
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
)
