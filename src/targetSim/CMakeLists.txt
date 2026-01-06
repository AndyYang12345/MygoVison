# 列出所有源文件
set(TARGET_SIM_SOURCES
    targetSim.cpp
    TrainingFrameGenerator.cpp
    PerformanceMonitor.cpp  # 新增PerformanceMonitor
)

# 列出所有头文件（用于IDE组织）
set(TARGET_SIM_HEADERS
    ${CMAKE_SOURCE_DIR}/include/targetSim/targetSim.hpp
    ${CMAKE_SOURCE_DIR}/include/targetSim/TrainingFrameGenerator.hpp
    ${CMAKE_SOURCE_DIR}/include/targetSim/PerformanceMonitor.hpp  # 新增
)

# 创建静态库 target_sim
add_library(target_sim STATIC ${TARGET_SIM_SOURCES})

# 设置包含目录
target_include_directories(target_sim
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include/targetSim>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# 链接OpenCV
target_link_libraries(target_sim PRIVATE ${OpenCV_LIBS})

# 添加编译定义（如果需要）
target_compile_definitions(target_sim PRIVATE TARGET_SIM_EXPORTS)

# 设置库属性
set_target_properties(target_sim PROPERTIES
    ARCHIVE_OUTPUT_NAME "target_sim"
    POSITION_INDEPENDENT_CODE ON
)

# 安装配置（可选）
# install(TARGETS target_sim
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin
# )

# 安装头文件（可选）
# install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/targetSim/
#     DESTINATION include/targetSim
#     FILES_MATCHING PATTERN "*.hpp"
# )

message(STATUS "Building target_sim library")
message(STATUS "Source files: ${TARGET_SIM_SOURCES}")
message(STATUS "Header files: ${TARGET_SIM_HEADERS}")