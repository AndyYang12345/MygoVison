cmake_minimum_required(VERSION 3.15)
project(target_sim_project VERSION 1.0.0 LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 查找OpenCV
find_package(OpenCV REQUIRED)

# 设置包含目录 - 关键步骤！
# 方法1：使用绝对路径（推荐）
include_directories(
    ${CMAKE_SOURCE_DIR}/include  # 包含targetSim.hpp的目录
    ${OpenCV_INCLUDE_DIRS}
)

# 或者方法2：更精确的设置
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/targetSim)
include_directories(${OpenCV_INCLUDE_DIRS})

# 打印调试信息
message(STATUS "Project source directory: ${CMAKE_SOURCE_DIR}")
message(STATUS "Include directories:")
message(STATUS "  - ${CMAKE_SOURCE_DIR}/include")
message(STATUS "  - ${OpenCV_INCLUDE_DIRS}")

# 设置编译选项
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 添加子目录
add_subdirectory(src/targetSim)
add_subdirectory(src/apps)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)