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
