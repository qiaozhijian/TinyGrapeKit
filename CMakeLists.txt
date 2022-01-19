cmake_minimum_required(VERSION 2.8.8)
project(FilterFusion)

# Set C++14
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# Set cmake
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Third party dependencies
find_package(Eigen3  REQUIRED)
find_package(GLOG  REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}/opencv4)
include_directories(${GLOG_INCLUDE_DIR})

include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${Pangolin_INCLUDE_DIRS}
)

# GeographicLib
add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/GeographicLib)
include_directories(${PROJECT_SOURCE_DIR}/Thirdparty/GeographicLib/include/)

# Make library
file(GLOB_RECURSE ALL_HEADER_FILES
    ${PROJECT_SOURCE_DIR}/include/*.h
)
file(GLOB_RECURSE ALL_SRC_FILES
    ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME} SHARED ${ALL_SRC_FILES} ${ALL_HEADER_FILES})
target_link_libraries(${PROJECT_NAME} pangolin libGeographiccc ${CERES_LIBRARIES})

## Declare C++ executables
add_executable(RunKAISTData example/RunKAISTData.cpp)
target_link_libraries(RunKAISTData glog ${OpenCV_LIBS} ${PROJECT_NAME})
