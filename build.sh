#!/bin/bash
# Build script for RK3588 aimbot

mkdir -p build && cd build
cmake ..
make -j$(nproc)
