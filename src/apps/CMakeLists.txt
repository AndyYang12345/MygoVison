# 创建 test_basic 可执行文件
add_executable(test_basic
    test_basic.cpp
)

target_link_libraries(test_basic
    target_sim
    ${OpenCV_LIBS}
)

target_include_directories(test_basic PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

# 设置可执行文件名称（可选）
set_target_properties(test_basic PROPERTIES
    OUTPUT_NAME "test_basic"
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# 创建 test_training 可执行文件
add_executable(test_training
    test_simple_training.cpp
)

target_link_libraries(test_training
    target_sim
    ${OpenCV_LIBS}
)

target_include_directories(test_training PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

set_target_properties(test_training PROPERTIES
    OUTPUT_NAME "test_training"
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# 如果需要，添加新的测试程序
# 注意：根据您提供的代码，test_basic.cpp 实际上包含的是 test_target_verification.cpp 的内容
# 如果您有多个测试文件，可以这样添加：
# add_executable(test_verification test_target_verification.cpp)
# target_link_libraries(test_verification target_sim ${OpenCV_LIBS})