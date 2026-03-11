#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <string>

#include "TargetSim/PentagonSimulator.hpp"
#include "TargetTracking/TargetTrackingPipeline.hpp"

int main() {
    // 模拟器配置
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

    // 跟踪管线配置
    TargetTrackingPipeline pipeline;
    PipelineConfig pipeline_cfg = pipeline.get_config();
    pipeline_cfg.fx = cam_cfg.fx;
    pipeline_cfg.fy = cam_cfg.fy;
    pipeline_cfg.cx = cam_cfg.cx;
    pipeline_cfg.cy = cam_cfg.cy;
    pipeline_cfg.pitch_home = 60.0f; // 向下旋转30度（90-30）
    pipeline_cfg.yaw_home = 105.0f;  // 向左旋转30度（135-30）
    pipeline_cfg.enable_serial = false;
    pipeline_cfg.serial_device = "/dev/ttyUSB0";
    pipeline_cfg.serial_baud = 115200;
    pipeline.set_config(pipeline_cfg);
    // 关闭跟踪器的调试输出
    TrackerConfig tracker_cfg = pipeline.get_tracker_config();
    tracker_cfg.show_debug_windows = false;
    tracker_cfg.print_debug_info = false;
    pipeline.set_tracker_config(tracker_cfg);

    if (pipeline_cfg.enable_serial && !pipeline.open_serial()) {
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

        cv::Mat canvas = output.canvas.empty() ? frame.clone() : output.canvas.clone();

        const bool gt_target_ok = simulator.has_last_target();
        const bool gt_laser_ok = simulator.has_last_laser();
        if (gt_target_ok) {
            cv::Point2f gt_target = simulator.get_last_target_position_projected();
            cv::circle(canvas, gt_target, 7, cv::Scalar(255, 0, 255), 2);
            cv::drawMarker(canvas, gt_target, cv::Scalar(255, 0, 255), cv::MARKER_CROSS, 18, 2);
        }
        if (gt_laser_ok) {
            cv::Point2f gt_laser = simulator.get_last_laser_position_projected();
            cv::circle(canvas, gt_laser, 9, cv::Scalar(0, 165, 255), 2);
            cv::circle(canvas, gt_laser, 4, cv::Scalar(0, 140, 255), -1);
        }

        constexpr float kHitThresholdPx = 30.0f;

        if (output.target_found && output.laser_found) {
            float detect_err = cv::norm(output.target_pos - output.laser_pos);
            const bool detect_hit = detect_err < kHitThresholdPx;
            cv::line(canvas,
                     cv::Point(static_cast<int>(output.target_pos.x), static_cast<int>(output.target_pos.y)),
                     cv::Point(static_cast<int>(output.laser_pos.x), static_cast<int>(output.laser_pos.y)),
                     detect_hit ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 255, 255),
                     2,
                     cv::LINE_AA);
            cv::putText(canvas,
                        detect_hit ? "DETECT HIT" : "DETECT MISS",
                        cv::Point(20, 340),
                        cv::FONT_HERSHEY_DUPLEX,
                        0.9,
                        detect_hit ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 200, 255),
                        2);
            cv::putText(canvas,
                        cv::format("Detect laser->target err: %.2f px", detect_err),
                        cv::Point(20, 250),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.6,
                        cv::Scalar(0, 255, 255),
                        2);
        } else {
            std::string reason = !output.target_found ? "target not found" : "laser not found";
            cv::putText(canvas,
                        ("Detect laser->target err: N/A (" + reason + ")").c_str(),
                        cv::Point(20, 250),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.55,
                        cv::Scalar(0, 180, 255),
                        2);
        }

        if (gt_target_ok && gt_laser_ok) {
            cv::Point2f gt_target = simulator.get_last_target_position_projected();
            cv::Point2f gt_laser = simulator.get_last_laser_position_projected();
            float gt_err = cv::norm(gt_target - gt_laser);
            const bool gt_hit = gt_err < kHitThresholdPx;
            cv::line(canvas,
                     cv::Point(static_cast<int>(gt_target.x), static_cast<int>(gt_target.y)),
                     cv::Point(static_cast<int>(gt_laser.x), static_cast<int>(gt_laser.y)),
                     gt_hit ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 180, 255),
                     2,
                     cv::LINE_AA);
            cv::putText(canvas,
                        cv::format("GT laser->target err: %.2f px", gt_err),
                        cv::Point(20, 280),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.6,
                        cv::Scalar(0, 200, 0),
                        2);
            cv::putText(canvas,
                        gt_hit ? "GT HIT" : "GT MISS",
                        cv::Point(20, 370),
                        cv::FONT_HERSHEY_DUPLEX,
                        0.9,
                        gt_hit ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 180, 255),
                        2);
        }

        cv::putText(canvas,
                    "Legend: magenta=GT target, orange=GT laser, yellow=detected laser",
                    cv::Point(20, 310),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(230, 230, 230),
                    1);

        cv::imshow(main_win, canvas);

        int key = cv::waitKey(30);
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }

        pipeline.handle_key(key);
    }

    return 0;
}
