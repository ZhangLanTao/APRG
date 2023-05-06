cmake_minimum_required(VERSION 3.25)
project(HirachicalPlaneSegmentation)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Open3D REQUIRED)
find_package(MKL REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR} ${MKL_INCLUDE} include include/rapidjson)

set(CMAKE_CXX_STANDARD 17)

if (CMAKE_BUILD_TYPE MATCHES "Release")
    message("-- Release mode")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3  -Wall")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3  -Wall")
    ADD_DEFINITIONS(-DRELEASE)
else ()
    message("-- Debug mode")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -Wall")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -Wall")
    ADD_DEFINITIONS(-DDEBUG)
endif ()

file(GLOB_RECURSE srcs src/*.cpp)

add_executable(HirachicalPlaneSegmentation ${srcs})

target_link_libraries(HirachicalPlaneSegmentation ${OpenCV_LIBS} Eigen3::Eigen Open3D::Open3D MKL::MKL OpenMP::OpenMP_CXX)