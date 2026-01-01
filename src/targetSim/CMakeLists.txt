# 创建静态库 target_sim
add_library(target_sim STATIC
    targetSim.cpp
    TrainingFrameGenerator.cpp
)

# 关键：设置包含目录 - 使用PUBLIC让依赖此库的目标也能访问头文件
target_include_directories(target_sim
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include  # 包含targetSim.hpp的目录
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)

# 链接OpenCV
target_link_libraries(target_sim PRIVATE ${OpenCV_LIBS})

# 打印调试信息
message(STATUS "Building target_sim library")
message(STATUS "Source files: targetSim.cpp TrainingFrameGenerator.cpp")
message(STATUS "Include dir for target_sim: ${CMAKE_SOURCE_DIR}/include")