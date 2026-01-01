# 应用程序目录的 CMakeLists.txt
# 基础功能测试
add_executable(test_basic
    test_basic.cpp
)

target_link_libraries(test_basic PRIVATE
    target_sim
    ${OpenCV_LIBS}
)

# 视频流测试
add_executable(test_stream
    test_stream.cpp
)

target_link_libraries(test_stream PRIVATE
    target_sim
    target_stream_sim
    ${OpenCV_LIBS}
)

# 安装可执行文件（可选）
install(TARGETS test_basic test_stream
    RUNTIME DESTINATION bin
)