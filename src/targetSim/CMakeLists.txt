# 库目录的 CMakeLists.txt
# 创建基础库：靶子仿真器
add_library(target_sim STATIC
    targetSim.cpp
)

target_include_directories(target_sim PUBLIC
    ${CMAKE_SOURCE_DIR}/include/targetSim
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(target_sim PUBLIC ${OpenCV_LIBS})

# 创建视频流模拟器库
add_library(target_stream_sim STATIC
    targetStreamSimulator.cpp
)

target_include_directories(target_stream_sim PUBLIC
    ${CMAKE_SOURCE_DIR}/include/targetSim
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(target_stream_sim PUBLIC 
    target_sim
    ${OpenCV_LIBS}
)

# 安装规则（可选）
install(TARGETS target_sim target_stream_sim
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/targetSim/
    DESTINATION include/targetSim
)