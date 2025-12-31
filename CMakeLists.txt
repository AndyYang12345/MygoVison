cmake_minimum_required(VERSION 3.12)
project(testTargetSimulation LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 1. 查找 OpenCV（这是PC仿真依赖的核心）
find_package(OpenCV REQUIRED)

# 2. 添加你的仿真库（共享库或静态库，这里用静态库更简单）
add_library(target_sim STATIC
    targetSim.cpp
    # 未来可以添加其他源文件，如 pid.cpp, gimbal_model.cpp
)

# 3. 为你的库指定头文件路径和链接库
target_include_directories(target_sim PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}  # 用于查找 targetSim.hpp
    ${OpenCV_INCLUDE_DIRS}       # OpenCV头文件路径
)
target_link_libraries(target_sim PUBLIC ${OpenCV_LIBS})

# 4. 添加主程序（用于测试你的库）
add_executable(test_target_simulation
    main.cpp  # 你的主程序，将调用 target_sim 库的功能
)

# 5. 链接你的库到主程序
target_link_libraries(test_target_simulation PRIVATE target_sim)

# 可选：如果 main.cpp 也需要直接包含 OpenCV，可以额外添加
# target_include_directories(test_target_simulation PRIVATE ${OpenCV_INCLUDE_DIRS})