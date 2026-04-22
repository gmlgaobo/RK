cmake_minimum_required(VERSION 3.10)
project(rk3588-aimbot C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# HDMI with OpenCV preview
find_package(OpenCV REQUIRED)
add_executable(hdmi_preview hdmi_capture.c hdmi_preview.cpp)
target_link_libraries(hdmi_preview pthread ${OpenCV_LIBS})

# USB HID mouse test
add_executable(hid_mouse_test usb_hid.c hid_mouse_test.c)
target_link_libraries(hid_mouse_test pthread)

# YOLOv8n-pose detection demo (requires rknn-toolkit2 and RGA)
include_directories(
  /home/ztl/toolkit2/rknn-toolkit2/rknpu2/runtime/Linux/librknn_api/include
  /home/ztl/github/RK/src
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(/home/ztl/toolkit2/rknn-toolkit2/rknpu2/runtime/Linux/librknn_api/aarch64)

# Check for RGA library
find_library(RGA_LIB rga PATHS /usr/lib)
if(RGA_LIB)
    message(STATUS "RGA library found: ${RGA_LIB}")
    add_definitions(-DUSE_RGA_PREPROCESS)
else()
    message(WARNING "RGA library not found, falling back to OpenCV preprocessing")
endif()

# TEMPORARILY FORCE OPENCV PREPROCESSING for testing
# Remove this line after verifying the model works with OpenCV preprocessing
add_definitions(-DFORCE_OPENCV_PREPROCESS)

add_executable(pose_demo hdmi_capture.c rga_preprocess.c usb_hid.c yolo_inference.cpp aim_engine.cpp hid_controller.cpp pose_demo.cpp)
if(RGA_LIB)
    target_link_libraries(pose_demo pthread ${OpenCV_LIBS} rknn_api rga)
else()
    target_link_libraries(pose_demo pthread ${OpenCV_LIBS} rknn_api)
endif()
