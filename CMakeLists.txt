cmake_minimum_required(VERSION 2.8)
project(Mono_vo)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# add cmake modules to use g2o
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

include_directories(
${PROJECT_SOURCE_DIR}/include
${Pangolin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${G2O_INCLUDE_DIRS}
${CSPARSE_INCLUDE_DIR}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED)
find_package(CSparse REQUIRED)

add_library(${PROJECT_NAME} SHARED
src/ORBDetectorAndMatching.cc
src/PoseEstimation.cc
src/Camera.cpp
src/Config.cpp
src/Frame.cpp
src/Initializer.cpp
src/Map.cpp
src/MapPoint.cpp
src/Config.cpp
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${Pangolin_LIBRARIES}
${EIGEN3_LIBS}
g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
${CSPARSE_LIBRARY}
)

add_executable(run src/run_Mono_vo.cpp)
target_link_libraries(run ${OpenCV_LIBS} ${PROJECT_NAME})

add_executable(pointcloud src/PointCloud.cpp)
target_link_libraries(pointcloud ${OpenCV_LIBS} ${PROJECT_NAME})

add_executable(Pnp src/PnpSolver.cpp)
target_link_libraries(Pnp ${OpenCV_LIBS} ${PROJECT_NAME})
