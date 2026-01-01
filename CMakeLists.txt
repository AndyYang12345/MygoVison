# CMakeLists.txt - 根目录
cmake_minimum_required(VERSION 3.12)
project(TargetSimulation LANGUAGES CXX)

# 项目基本信息
set(PROJECT_NAME "TargetSimulation")
set(PROJECT_VERSION "1.0.0")
set(PROJECT_DESCRIPTION "视觉云台激光追踪仿真系统")

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置编译类型（默认Release）
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# 输出目录设置
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# 设置包含目录
include_directories(${CMAKE_SOURCE_DIR}/include)

# 查找OpenCV（必需）
find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV: ${OpenCV_VERSION}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")

# 配置选项
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
option(BUILD_TESTS "Build tests" ON)

# 添加子目录
add_subdirectory(src/targetSim)

if(BUILD_TESTS)
    add_subdirectory(src/apps)
endif()



# 安装配置
install(DIRECTORY include/targetSim DESTINATION include)
install(TARGETS target_sim DESTINATION lib)

# 总结信息
message(STATUS "==========================================")
message(STATUS "项目配置完成:")
message(STATUS "  项目名称: ${PROJECT_NAME}")
message(STATUS "  版本: ${PROJECT_VERSION}")
message(STATUS "  构建类型: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++标准: ${CMAKE_CXX_STANDARD}")
message(STATUS "  构建测试: ${BUILD_TESTS}")
message(STATUS "  构建示例: ${BUILD_EXAMPLES}")
message(STATUS "  OpenCV版本: ${OpenCV_VERSION}")
message(STATUS "==========================================")