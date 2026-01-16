# src/apps/CMakeLists.txt
add_executable(test_basic test_basic.cpp)
target_link_libraries(test_basic PRIVATE target_tracker target_sim ${OpenCV_LIBS})
set_target_properties(test_basic PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_executable(test_training test_simple_training.cpp)
target_link_libraries(test_training PRIVATE target_tracker target_sim ${OpenCV_LIBS})
set_target_properties(test_training PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_executable(test_threshold_tuner test_threshold_tuner.cpp)
target_link_libraries(test_threshold_tuner PRIVATE target_tracker target_sim ${OpenCV_LIBS})
set_target_properties(test_threshold_tuner PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_executable(test_single_image
    test_single_image.cpp
    TestUtils.cpp
)

target_link_libraries(test_single_image PRIVATE target_tracker ${OpenCV_LIBS})
set_target_properties(test_single_image PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
# 编译选项
target_compile_options(test_basic PRIVATE -g -Wall -Wextra)
target_compile_options(test_training PRIVATE -g -Wall -Wextra)
target_compile_options(test_threshold_tuner PRIVATE -g -Wall -Wextra)