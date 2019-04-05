cmake_minimum_required(VERSION 2.8)
project(fish2sphere)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

find_package(OpenCV 3.0 QUIET)
find_package(Eigen3)

include_directories(./bitmaplib.h ${EIGEN3_INCLUDE_DIR})

LINK_DIRECTORIES("/usr/lib/x86_64-linux-gnu/")
camodocal_executable(fish2sphere
        fish2sphere.cc bitmaplib.cc)

camodocal_link_libraries(fish2sphere
        ${OpenCV_LIBS}
        libjpeg.so
        libtiff.so
        )
