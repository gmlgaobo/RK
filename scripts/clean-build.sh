#!/bin/bash

echo "==================================="
echo "RK3588 AimBot 项目清理脚本"
echo "==================================="
echo ""

read -p "确认清理？这将删除编译产物和临时文件 [y/N] " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "取消清理"
    exit 1
fi

echo ""
echo "清理编译产物..."
rm -rf build/
echo "✓ build/ 已删除"

echo ""
echo "清理临时文件..."
find . -name "*.o" -type f -delete
find . -name "*.a" -type f -delete
find . -name "*.log" -type f -delete
find . -name "*~" -type f -delete
echo "✓ 临时文件已删除"

echo ""
echo "清理 CMake 缓存..."
find . -name "CMakeCache.txt" -type f -delete
find . -name "CMakeFiles" -type d -exec rm -rf {} + 2>/dev/null
find . -name "cmake_install.cmake" -type f -delete
find . -name "Makefile" -type f -path "*/build/*" -delete
echo "✓ CMake 缓存已删除"

echo ""
echo "==================================="
echo "清理完成！"
echo "==================================="
echo ""
echo "保留的重要文件："
echo "  - src/          源代码"
echo "  - scripts/      配置脚本"
echo "  - config.ini    配置文件"
echo "  - docs/         文档"
echo ""
echo "重新编译："
echo "  bash build.sh"
echo ""
