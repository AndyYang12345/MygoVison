# 根目录 CMakeLists.txt
cmake_minimum_required(VERSION 3.12)
project(TargetSimulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 设置包含目录
include_directories(${CMAKE_SOURCE_DIR}/include)

# 查找依赖
find_package(OpenCV REQUIRED)

# 添加子目录
add_subdirectory(src/targetSim)
add_subdirectory(src/apps)