cmake_minimum_required(VERSION 3.10)
project(rk3588-aimbot LANGUAGES C CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_C_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

add_subdirectory(src)
