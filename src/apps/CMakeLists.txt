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
)

target_link_libraries(test_single_image PRIVATE target_tracker ${OpenCV_LIBS})
set_target_properties(test_single_image PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")


add_executable(pentagon_3d_perspective pentagon_3d_perspective.cpp)
target_link_libraries(pentagon_3d_perspective PRIVATE target_tracker target_sim ${OpenCV_LIBS})
set_target_properties(pentagon_3d_perspective PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_executable(pentagon_simulator_app pentagon_simulator_app.cpp)
target_link_libraries(pentagon_simulator_app PRIVATE target_tracker target_sim ${OpenCV_LIBS})
set_target_properties(pentagon_simulator_app PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_executable(camera_canvas_demo camera_canvas_demo.cpp)
target_link_libraries(camera_canvas_demo PRIVATE target_sim ${OpenCV_LIBS})
set_target_properties(camera_canvas_demo PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_executable(test_tracker_sim test_tracker_sim.cpp)
target_link_libraries(test_tracker_sim PRIVATE maixcdk_target_tracker target_sim ${OpenCV_LIBS})
set_target_properties(test_tracker_sim PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_executable(test_gimbal_control test_gimbal_control.cpp)
target_link_libraries(test_gimbal_control PRIVATE target_tracker ${OpenCV_LIBS})
set_target_properties(test_gimbal_control PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_executable(test_target3d_generator test_target3d_generator.cpp)
target_link_libraries(test_target3d_generator PRIVATE target_sim ${OpenCV_LIBS})
set_target_properties(test_target3d_generator PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_executable(ga_training GAtraining.cpp)
target_link_libraries(ga_training PRIVATE target_sim ${OpenCV_LIBS})
set_target_properties(ga_training PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
