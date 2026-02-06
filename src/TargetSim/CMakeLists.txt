# src/TargetSim/CMakeLists.txt
set(TARGET_SIM_SOURCES
    TargetSim.cpp
    TrainingFrameGenerator.cpp
    PerformanceMonitor.cpp
    SimulationCamera.cpp
    PentagonSimulator.cpp
    Target3DGenerator.cpp
)

add_library(target_sim ${TARGET_SIM_SOURCES})

# 提供头文件路径
target_include_directories(target_sim PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
)

# ✅ 使用主项目传递进来的 OpenCV
target_link_libraries(target_sim PRIVATE ${OpenCV_LIBS})

set_target_properties(target_sim PROPERTIES
    OUTPUT_NAME "target_sim"
    POSITION_INDEPENDENT_CODE ON
)

message(STATUS "✅ target_sim linked with OpenCV: ${OpenCV_LIBS}")
