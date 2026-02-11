#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>
#include <string>

#include "TargetSim/PentagonSimulator.hpp"
#include "TargetTracking/TargetTrackingPipeline.hpp"

int main() {
    const int width = 640;
    const int height = 640;
    PentagonSimulator::CameraConfig cam_cfg(width, height, 30.0f);
    cam_cfg.fx = 381.625f;
    cam_cfg.fy = 381.625f;
    cam_cfg.cx = width * 0.5f;
    cam_cfg.cy = height * 0.5f;
    cam_cfg.position = cv::Point3f(0.0f, 0.0f, 1000.0f);
    cam_cfg.pitch = 0.0f;
    cam_cfg.yaw = 0.0f;
    PentagonSimulator simulator(cam_cfg);
    simulator.resume();

    TargetTrackingPipeline pipeline;
    PipelineConfig pipeline_cfg = pipeline.get_config();
    pipeline_cfg.fx = cam_cfg.fx;
    pipeline_cfg.fy = cam_cfg.fy;
    pipeline_cfg.cx = cam_cfg.cx;
    pipeline_cfg.cy = cam_cfg.cy;
    pipeline_cfg.pitch_home = 60.0f; // 向下旋转30度（90-30）
    pipeline_cfg.yaw_home = 105.0f;  // 向左旋转30度（135-30）
    pipeline_cfg.enable_serial = true;
    pipeline_cfg.serial_device = "/dev/ttyUSB0";
    pipeline_cfg.serial_baud = 115200;
    pipeline.set_config(pipeline_cfg);

    TrackerConfig tracker_cfg = pipeline.get_tracker_config();
    tracker_cfg.show_debug_windows = false;
    tracker_cfg.print_debug_info = false;
    pipeline.set_tracker_config(tracker_cfg);

    if (!pipeline.open_serial()) {
        std::cerr << "Failed to open /dev/ttyUSB0 at 115200." << std::endl;
    }

    const std::string main_win = "Gimbal Control";
    cv::namedWindow(main_win, cv::WINDOW_AUTOSIZE);

    auto last_tick = std::chrono::steady_clock::now();

    while (true) {
        auto now_tick = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now_tick - last_tick).count();
        last_tick = now_tick;

        float pitch_rad = (pipeline.get_pitch_angle() - 90.0f) * static_cast<float>(CV_PI) / 180.0f;
        float yaw_rad = (pipeline.get_yaw_angle() - 135.0f) * static_cast<float>(CV_PI) / 180.0f;
        simulator.set_camera_pitch(pitch_rad);
        simulator.set_camera_yaw(yaw_rad);

        cv::Mat frame = simulator.get_frame();
        PipelineOutput output = pipeline.process_frame(frame, dt);

        cv::imshow(main_win, output.canvas);

        int key = cv::waitKey(30);
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }

        pipeline.handle_key(key);
    }

    return 0;
}
