cmake_minimum_required(VERSION 3.0.2)
project(DVL_SLAM_ROS)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package(catkin REQUIRED COMPONENTS
  image_transport
  laser_geometry
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  cv_bridge
  nav_msgs
  octomap_msgs
)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(octomap REQUIRED)

find_package(PCL REQUIRED)
message("[PCL Version] : ${PCL_VERSION}")
message("[PCL path] : ${PCL_INCLUDE_DIRS}")

find_package(Eigen3 3.3 REQUIRED)
message("{Eigen path} : ${EIGEN3_INCLUDE_DIR}")
message("[Eigen Version] : ${Eigen3_VERSION}")

find_package(Sophus REQUIRED)
#find_package(DLib REQUIRED)
#find_package(DBoW2 REQUIRED)
#find_library(cholmod REQUIRED)
find_package(G2O REQUIRED)


catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Sophus_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
#  ${CHOLMOD_INCLUDE_DIR}
  ${G2O_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}

)

add_executable(dvl_slam_modify
  src/main.cpp
  src/Config.cpp
  src/Frame.cpp
  src/Tracker.cpp
  src/GraphOptimizer.cpp
  src/Sensor.cpp
  src/System.cpp
  src/KeyFrame.cpp
  src/PinholeModel.cpp
  src/SensorRos.cpp
  src/SensorSavedData.cpp
  src/Logger.cpp
)

target_link_libraries(dvl_slam_modify
  ${catkin_LIBRARIES}
  yaml-cpp
 # ${DBoW2_LIBS}
  ${PCL_LIBRARIES}
  ${DLib_LIBS}
  ${CHOLMOD_LIBRARIES}
  ${G2O_STUFF_LIBRARY}
  ${G2O_CORE_LIBRARY}
  ${G2O_SOLVER_DENSE}
  ${G2O_SOLVER_EIGEN}
  ${G2O_SOLVER_CHOLMOD}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SBA}
  ${OpenCV_LIBRARIES}
  ${OpenCV_LIBS}
  ${OCTOMAP_LIBRARIES}
)
