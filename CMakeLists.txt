cmake_minimum_required(VERSION 3.25)
project(HierarchicalPlaneSegmentation)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Open3D REQUIRED)
find_package(MKL REQUIRED)
#find_package(OpenMP REQUIRED)

if(OPENMP_FOUND)
    message("-- OpenMP found")
    ADD_DEFINITIONS(-DENABLE_OMP)
    set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS})
    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS})
    set(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS})
    target_link_libraries(OpenMP::OpenMP_CXX)
endif()

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


#set flags for use gprof
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -pg")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")


file(GLOB_RECURSE srcs src/*.cpp)

add_executable(HierarchicalPlaneSegmentation ${srcs})

target_link_libraries(HierarchicalPlaneSegmentation ${OpenCV_LIBS} Eigen3::Eigen Open3D::Open3D MKL::MKL)