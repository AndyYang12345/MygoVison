# src/targetSim/CMakeLists.txt
# 靶子仿真核心库

# 设置库名称
set(LIB_NAME target_sim)

# 源文件列表
set(SOURCES
    targetSim.cpp
    TrainingFrameGenerator.cpp
)

# 头文件目录
set(INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/include/targetSim
    ${OpenCV_INCLUDE_DIRS}
)

# 创建库
if(BUILD_SHARED_LIBS)
    add_library(${LIB_NAME} SHARED ${SOURCES})
    message(STATUS "构建共享库: ${LIB_NAME}")
else()
    add_library(${LIB_NAME} STATIC ${SOURCES})
    message(STATUS "构建静态库: ${LIB_NAME}")
endif()

# 设置包含目录
target_include_directories(${LIB_NAME} PUBLIC ${INCLUDE_DIRS})

# 链接库
target_link_libraries(${LIB_NAME} PUBLIC ${OpenCV_LIBS})

# 设置库属性
set_target_properties(${LIB_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    OUTPUT_NAME ${LIB_NAME}
    ARCHIVE_OUTPUT_NAME ${LIB_NAME}
    LIBRARY_OUTPUT_NAME ${LIB_NAME}
)

# 安装目标
install(TARGETS ${LIB_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

message(STATUS "  添加库: ${LIB_NAME}")
message(STATUS "  源文件: ${SOURCES}")