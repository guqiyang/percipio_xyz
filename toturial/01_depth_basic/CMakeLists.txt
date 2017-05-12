cmake_minimum_required (VERSION 2.8.11)
project (01_depth_basic)

find_package(OpenCV COMPONENTS core ml imgproc highgui video contrib REQUIRED)
#find_package(camm REQUIRED)

ADD_DEFINITIONS(
    -std=c++11 
    -g
)

add_executable (01_depth_basic
    main.cpp
    depth_render.cpp
)


#add_subdirectory ()
#add_library ()
#target_include_directories ()
target_link_libraries (01_depth_basic
    libopencv_core.so
    libopencv_ml.so
    libopencv_imgproc.so 
    libopencv_highgui.so 
    libopencv_video.so 
    libopencv_contrib.so 
    libcamm.so
)
