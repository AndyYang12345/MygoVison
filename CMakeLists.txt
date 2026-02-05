# Mygo/CMakeLists.txt
cmake_minimum_required(VERSION 3.14)
project(Mygo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 启用编译命令输出（用于 IDE）
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# =============== ✅ 关键修复：只在这里查找 OpenCV ===============
find_package(OpenCV REQUIRED core imgproc highgui)

if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found! Please install libopencv-dev")
endif()

# 输出调试信息
message(STATUS "🎯 Found OpenCV version: ${OpenCV_VERSION}")
message(STATUS "📚 OpenCV libraries: ${OpenCV_LIBS}")

# 允许所有子模块访问 include/
include_directories(${CMAKE_SOURCE_DIR}/include)

# ======== MaixCDK vision_target_tracking (OpenCV-only) ========
set(MAIXCDK_DIR "/home/harekasa/MaixCDK")

add_library(maixcdk_target_tracker
    ${MAIXCDK_DIR}/components/vision_target_tracking/src/maix_target_tracker.cpp
)

target_include_directories(maixcdk_target_tracker PUBLIC
    ${MAIXCDK_DIR}/components/vision_target_tracking/include
)

target_compile_definitions(maixcdk_target_tracker PUBLIC
    MAIX_VISION_TARGET_TRACKER_NO_IMAGE
)

target_link_libraries(maixcdk_target_tracker PRIVATE ${OpenCV_LIBS})

# 导出 OpenCV 到子目录（可选，因为是全局变量）
# set(OpenCV_LIBS ${OpenCV_LIBS} PARENT_SCOPE)

# 添加子模块
add_subdirectory(src/TargetTracking)
add_subdirectory(src/TargetSim)
add_subdirectory(src/apps)
message(STATUS "🚀 Mygo build configuration complete.")