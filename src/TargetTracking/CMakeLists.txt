# 列出所有源文件
set(TARGET_SIM_SOURCES
    TargetTracker.cpp
)

# 列出所有头文件（用于IDE组织）
set(TARGET_SIM_HEADERS
    ${CMAKE_SOURCE_DIR}/include/TargetTracking/TargetTracker.hpp
    ${CMAKE_SOURCE_DIR}/include/TargetSim/TrainingFrameGenerator.hpp
)

# 创建静态库 target_tracker
add_library(target_tracker STATIC ${TARGET_SIM_SOURCES})

# 设置包含目录
target_include_directories(target_tracker
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/TargetSim>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/TargetTracking>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# 链接OpenCV
target_link_libraries(target_tracker PRIVATE ${OpenCV_LIBS})

# 添加编译定义（如果需要）
target_compile_definitions(target_tracker PRIVATE TARGET_TRACKER_EXPORTS)

# 设置库属性
set_target_properties(target_tracker PROPERTIES
    ARCHIVE_OUTPUT_NAME "target_tracker"
    POSITION_INDEPENDENT_CODE ON
)

# 安装配置（可选）
# install(TARGETS target_tracker
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin
# )

# 安装头文件（可选）
# install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/TargetTracking/
#     DESTINATION include/TargetTracking
#     FILES_MATCHING PATTERN "*.hpp"
# )

message(STATUS "Building target_tracker library")
message(STATUS "Source files: ${TARGET_SIM_SOURCES}")
message(STATUS "Header files: ${TARGET_SIM_HEADERS}")