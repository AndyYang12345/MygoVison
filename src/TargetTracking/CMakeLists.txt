# src/TargetTracking/CMakeLists.txt
set(TARGET_TRACKER_SOURCES
    TargetTracker.cpp
    SerialPort.cpp
    GimbalControl.cpp
    TargetTrackingPipeline.cpp
)

add_library(target_tracker ${TARGET_TRACKER_SOURCES})

# 提供头文件路径
target_include_directories(target_tracker PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
)

# ✅ 直接使用主项目找到的 OpenCV 库
target_link_libraries(target_tracker PRIVATE ${OpenCV_LIBS})

# KalmanFilter 在 opencv_video 中
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui video)
target_link_libraries(target_tracker PRIVATE opencv_video)

# 连接 MaixCDK Vision（若存在）
if(TARGET maixcdk_vision)
    target_link_libraries(target_tracker PRIVATE maixcdk_vision)
endif()

set_target_properties(target_tracker PROPERTIES
    OUTPUT_NAME "target_tracker"
    POSITION_INDEPENDENT_CODE ON
)

message(STATUS "✅ target_tracker linked with OpenCV: ${OpenCV_LIBS}")
