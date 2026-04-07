#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>

#include "TargetSim/PentagonSimulator.hpp"
#include "TargetTracking/TargetTrackingPipeline.hpp"

namespace {

const char* to_state_name(TrackState state) {
    switch (state) {
        case TrackState::Waiting: return "Waiting";
        case TrackState::Searching: return "Searching";
        case TrackState::Locked: return "Locked";
        case TrackState::Tracking: return "Tracking";
        default: return "Unknown";
    }
}

} // namespace

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
    cam_cfg.pitch_axis_below_optical_center_mm = 100.0f;
    cam_cfg.yaw_axis_behind_pitch_axis_mm = 70.0f;
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
    pipeline_cfg.yaw_home = 270.0f;  // 初始位设为yaw上限，便于观察边界扫描
    pipeline_cfg.pitch_pwm_zero_angle = pipeline_cfg.pitch_home;
    pipeline_cfg.yaw_pwm_zero_angle = pipeline_cfg.yaw_home;
    pipeline_cfg.pid_kp = 15.40262f;
    pipeline_cfg.pid_ki = 0.05f;
    pipeline_cfg.pid_kd = 0.307062f;
    pipeline_cfg.enable_serial = true;
    pipeline_cfg.serial_device = "/dev/ttyACM0";
    pipeline_cfg.serial_baud = 115200;
    pipeline.set_config(pipeline_cfg);
    // 关闭跟踪器的调试输出
    TrackerConfig tracker_cfg = pipeline.get_tracker_config();
    tracker_cfg.show_debug_windows = false;
    tracker_cfg.print_debug_info = false;
    pipeline.set_tracker_config(tracker_cfg);

    const bool auto_start_search = true;
    const bool auto_start_tracking = true;
    const std::string debug_log_path = "gimbal_control_debug.csv";
    std::ofstream debug_log(debug_log_path, std::ios::out | std::ios::trunc);
    if (!debug_log.is_open()) {
        std::cerr << "Failed to open debug log file: " << debug_log_path << std::endl;
    } else {
        debug_log << "time_sec,frame,state,event,target_x,target_y,roi_active,roi_x,roi_y,roi_w,roi_h,"
                  << "aim_x,aim_y,aim_source,laser_x,laser_y,pitch_deg,yaw_deg,pitch_speed,yaw_speed,"
                  << "lock_count,lost_count,command\n";
        std::cout << "Debug log file: " << debug_log_path << std::endl;
    }

    if (pipeline_cfg.enable_serial) {
        if (!pipeline.open_serial()) {
            std::cerr << "Failed to open " << pipeline_cfg.serial_device
                      << " at " << pipeline_cfg.serial_baud << std::endl;
        } else {
            std::cout << "Serial opened: " << pipeline_cfg.serial_device
                      << " @ " << pipeline_cfg.serial_baud << std::endl;
            const std::string init_cmd = "{P1500T1000P1350T1000P2300T1000P1500T1000P1500T1000}";
            bool init_ok = pipeline.send_raw_serial_command(init_cmd);
            std::cout << "Init cmd sent: " << (init_ok ? "true" : "false") << std::endl;
        }
    }

    const std::string main_win = "Gimbal Control";
    cv::namedWindow(main_win, cv::WINDOW_AUTOSIZE);

    auto last_tick = std::chrono::steady_clock::now();
    const auto run_start_tick = last_tick;
    int frame_index = 0;
    TrackState last_state = TrackState::Waiting;

    const float sim_pitch_neutral_deg = 60.0f;
    const float sim_yaw_neutral_deg = 105.0f;

    if (auto_start_search) {
        pipeline.handle_key(' ');
        std::cout << "[AUTO] Waiting -> Searching (startup)" << std::endl;
    }

    while (true) {
        auto now_tick = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now_tick - last_tick).count();
        last_tick = now_tick;

        float pitch_rad = (pipeline.get_pitch_angle() - sim_pitch_neutral_deg) * static_cast<float>(CV_PI) / 180.0f;
        float yaw_rad = (pipeline.get_yaw_angle() - sim_yaw_neutral_deg) * static_cast<float>(CV_PI) / 180.0f;
        simulator.set_camera_pitch(pitch_rad);
        simulator.set_camera_yaw(yaw_rad);

        cv::Mat frame = simulator.get_frame();
        PipelineOutput output = pipeline.process_frame(frame, dt);
        frame_index++;

        std::string frame_event;
        if (output.state != last_state) {
            frame_event = std::string("STATE_CHANGE:") + to_state_name(last_state) + "->" + to_state_name(output.state);
            std::cout << "[STATE] " << frame_event << " at frame " << frame_index << std::endl;
        }

        if (auto_start_tracking && output.state == TrackState::Locked) {
            pipeline.handle_key(' ');
            if (!frame_event.empty()) {
                frame_event += "|";
            }
            frame_event += "AUTO_SWITCH:Locked->Tracking";
            std::cout << "[AUTO] Locked -> Tracking at frame " << frame_index << std::endl;
        }

        if (debug_log.is_open()) {
            const float time_sec = std::chrono::duration<float>(now_tick - run_start_tick).count();
            debug_log << std::fixed << std::setprecision(6)
                      << time_sec << ","
                      << frame_index << ","
                      << to_state_name(output.state) << ","
                      << (frame_event.empty() ? "-" : frame_event) << ","
                      << output.target_pos.x << ","
                      << output.target_pos.y << ","
                      << (output.roi_active ? 1 : 0) << ","
                      << output.roi_rect.x << ","
                      << output.roi_rect.y << ","
                      << output.roi_rect.width << ","
                      << output.roi_rect.height << ","
                      << output.aim_pos.x << ","
                      << output.aim_pos.y << ","
                      << (output.aim_from_laser ? "laser" : "center") << ","
                      << output.laser_pos.x << ","
                      << output.laser_pos.y << ","
                      << output.pitch_angle << ","
                      << output.yaw_angle << ","
                      << output.pitch_speed << ","
                      << output.yaw_speed << ","
                      << output.lock_count << ","
                      << output.lost_count << ",\""
                      << output.command << "\"\n";
            debug_log.flush();
        }

        last_state = output.state;

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

        if (key == ' ') {
            std::cout << "[MANUAL] SPACE pressed, request state transition" << std::endl;
        }
        pipeline.handle_key(key);
    }

    if (debug_log.is_open()) {
        debug_log.close();
    }

    return 0;
}
