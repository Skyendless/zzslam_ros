
cmake_minimum_required(VERSION 3.1)
project(zzslam_ros)

set(ZZ_SLAM_DIR
   $ENV{HOME}/workspace/zzslam
)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    )

find_package(Eigen3 REQUIRED)
find_package(CSparse REQUIRED)
find_package(G2O REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Glog REQUIRED)  # glog

catkin_package()

# 添加头文件
include_directories(
	${ZZ_SLAM_DIR}/include
    ${PROJECT_SOURCE_DIR}/include
	${OpenCV_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${Sophus_INCLUDE_DIRS}
	${G2O_INCLUDE_DIRS}
	${CSPARSE_INCLUDE_DIR}
	${GLOG_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	)

# set lib
set(LIBS
    ${ZZ_SLAM_DIR}/lib/libzzslam.so
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    ${Pangolin_LIBRARIES}
    # ${G2O_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
    ${GLOG_LIBRARIES}
    ${CSPARSE_LIBRARY}
    pthread
    fmt
)

add_executable(zzslam_ros src/zzslam_ros.cpp)
target_link_libraries(zzslam_ros ${LIBS})

add_executable(vedio_pub_node src/vedio_pub_node.cpp)
target_link_libraries(vedio_pub_node ${OpenCV_LIBS} ${catkin_LIBRARIES})


