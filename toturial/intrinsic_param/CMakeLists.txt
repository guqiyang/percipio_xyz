cmake_minimum_required (VERSION 2.8.11)
project (02_point_cloud)

find_package(PCL 1.7 COMPONENTS common visualization REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED)
#find_package(camm REQUIRED)

ADD_DEFINITIONS(
    -std=c++11 
    -g
    -DHAVE_PCL
)

add_executable (get_camera_intrinsic
    depth_render.cpp 
    getCameraIntrinsic.cpp
)

include_directories(
    /usr/include/pcl-1.7
    /usr/include/eigen3
    /usr/include/vtk-5.8
)

#add_subdirectory ()
#add_library ()
#target_include_directories ()
target_link_libraries (get_camera_intrinsic
    ${OpenCV_LIBS}
    pcl_common
    pcl_visualization
    boost_system
    boost_thread
    camm.so
)
