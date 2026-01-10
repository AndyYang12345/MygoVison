# 设置应用程序目录
set(APPS_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# test_basic 可执行文件
add_executable(test_basic
    test_basic.cpp
)

target_link_libraries(test_basic
    target_sim
    ${OpenCV_LIBS}
)

# test_training 可执行文件
add_executable(test_training
    test_simple_training.cpp
)

target_link_libraries(test_training
    target_sim
    target_tracker
    ${OpenCV_LIBS}
)

# 设置所有可执行文件的属性
foreach(target_name test_basic test_training)
    set_target_properties(${target_name} PROPERTIES
        OUTPUT_NAME "${target_name}"
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    )
endforeach()

# 添加新的测试程序（如果您有其他测试）
# add_executable(test_performance test_performance.cpp)
# target_link_libraries(test_performance target_sim ${OpenCV_LIBS})
# set_target_properties(test_performance PROPERTIES
#     OUTPUT_NAME "test_performance"
#     RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
# )