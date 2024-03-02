cmake_minimum_required(VERSION 3.0.2)

project(stereo_slam)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  visualization_msgs
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

find_package(FMT REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES helloworld
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


############### dependencies ######################
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/stereo_slam
  "/usr/include/eigen3"
)

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${CSPARSE_LIBRARY}
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        pthread
        )

aux_source_directory(./src/stereo_slam SRCS)

add_executable(run_stereo src/run_stereo.cpp ${SRCS})

target_link_libraries(run_stereo ${catkin_LIBRARIES})
target_link_libraries(run_stereo ${THIRD_PARTY_LIBS})
target_link_libraries(run_stereo fmt::fmt)
