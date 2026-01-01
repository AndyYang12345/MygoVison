# src/tests/CMakeLists.txt
# 测试程序

# 基础功能测试
add_executable(test_basic
    test_basic.cpp
)

target_link_libraries(test_basic PRIVATE
    target_sim
    ${OpenCV_LIBS}
)

# 设置输出名称
set_target_properties(test_basic PROPERTIES
    OUTPUT_NAME "test_basic"
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/tests
)

# 训练帧生成器测试
add_executable(test_training
    test_simple_training.cpp
)

target_link_libraries(test_training PRIVATE
    target_sim
    ${OpenCV_LIBS}
)

set_target_properties(test_training PROPERTIES
    OUTPUT_NAME "test_training"
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/tests
)

message(STATUS "  添加测试程序:")
message(STATUS "    - test_basic")
message(STATUS "    - test_training")