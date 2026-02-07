#include <opencv2/opencv.hpp>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>

#include "TargetSim/PentagonSimulator.hpp"
#include "TargetTracking/GimbalControl.hpp"
#include "TargetTracking/TargetTracker.hpp"

namespace {
struct PID {
    float kp{11.8354f};
    float ki{0.315478f};
    float kd{0.0215511f};
    float integral{0.0f};
    float prev_error{0.0f};
    bool has_prev{false};
};

float clamp_value(float v, float lo, float hi) {
    return std::max(lo, std::min(v, hi));
}

float pid_step(float error, float dt, PID& pid, float integral_limit) {
    pid.integral += error * dt;
    pid.integral = clamp_value(pid.integral, -integral_limit, integral_limit);

    float derivative = 0.0f;
    if (pid.has_prev && dt > 1e-6f) {
        derivative = (error - pid.prev_error) / dt;
    }
    pid.prev_error = error;
    pid.has_prev = true;

    return pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
}

} // namespace

int main() {
    GimbalControl gimbal;

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
    simulator.pause();

    TargetTracker tracker;
    TrackerConfig tracker_cfg = tracker.get_config();
    tracker_cfg.show_debug_windows = false;
    tracker_cfg.print_debug_info = false;
    tracker.set_config(tracker_cfg);

    const std::string main_win = "Gimbal Control";
    cv::namedWindow(main_win, cv::WINDOW_AUTOSIZE);

    float pitch_angle = 90.0f;
    float yaw_angle = 135.0f;

    float pitch_speed = 0.0f;
    float yaw_speed = 0.0f;

    PID pid_pitch;
    PID pid_yaw;

    const float max_speed = 180.0f; // deg/s
    const float integral_limit = 30.0f;
    const float settle_threshold = 0.8f; // deg
    const float settle_hold = 0.2f; // sec
    float settle_timer = 0.0f;
    bool rotating = false;

    auto last_tick = std::chrono::steady_clock::now();

    while (true) {
        auto now_tick = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now_tick - last_tick).count();
        last_tick = now_tick;
        dt = clamp_value(dt, 0.001f, 0.05f);

        float pitch_rad = (pitch_angle - 90.0f) * static_cast<float>(CV_PI) / 180.0f;
        float yaw_rad = (yaw_angle - 135.0f) * static_cast<float>(CV_PI) / 180.0f;
        simulator.set_camera_pitch(pitch_rad);
        simulator.set_camera_yaw(yaw_rad);

        cv::Mat frame = simulator.get_frame();
        cv::Mat canvas = frame.clone();

        TargetInfo info = tracker.process_frame(frame);
        cv::Point2f target_pos(-1.0f, -1.0f);
        if (info.found) {
            target_pos = info.target_center;
        }

        if (rotating && target_pos.x >= 0.0f) {
            float dx = target_pos.x - cam_cfg.cx;
            float dy = target_pos.y - cam_cfg.cy;
            float pitch_error = std::atan2(dy, cam_cfg.fy) * 180.0f / static_cast<float>(CV_PI);
            float yaw_error = -std::atan2(dx, cam_cfg.fx) * 180.0f / static_cast<float>(CV_PI);

            std::cout << std::fixed << std::setprecision(2)
                      << "Target(px):(" << target_pos.x << "," << target_pos.y << ")"
                      << " dx,dy:(" << dx << "," << dy << ")"
                      << " pitch_err:" << pitch_error
                      << " yaw_err:" << yaw_error
                      << " cur_pitch:" << pitch_angle
                      << " cur_yaw:" << yaw_angle
                      << std::endl;

            float pitch_speed_cmd = pid_step(pitch_error, dt, pid_pitch, integral_limit);
            float yaw_speed_cmd = pid_step(yaw_error, dt, pid_yaw, integral_limit);

            pitch_speed = clamp_value(pitch_speed_cmd, -max_speed, max_speed);
            yaw_speed = clamp_value(yaw_speed_cmd, -max_speed, max_speed);

            pitch_angle += pitch_speed * dt;
            yaw_angle += yaw_speed * dt;

            if (std::abs(pitch_error) < settle_threshold && std::abs(yaw_error) < settle_threshold) {
                settle_timer += dt;
                if (settle_timer >= settle_hold) {
                    rotating = false;
                    settle_timer = 0.0f;
                    pid_pitch = {};
                    pid_yaw = {};
                }
            } else {
                settle_timer = 0.0f;
            }
        } else {
            pitch_speed = 0.0f;
            yaw_speed = 0.0f;
        }

        gimbal.set_pitch_angle(pitch_angle);
        gimbal.set_yaw_angle(yaw_angle);
        gimbal.set_pitch_speed(std::abs(pitch_speed));
        gimbal.set_yaw_speed(std::abs(yaw_speed));
        gimbal.get_command();

        std::ostringstream status;
        status << std::fixed << std::setprecision(2)
               << "Pitch: " << pitch_angle << " deg  |  Yaw: " << yaw_angle << " deg";
        cv::putText(canvas, status.str(), {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(220, 220, 220), 2);

        std::ostringstream speed_line;
        speed_line << std::fixed << std::setprecision(2)
               << "Pitch Speed: " << pitch_speed << " deg/s  |  "
               << "Yaw Speed: " << yaw_speed << " deg/s";
        cv::putText(canvas, speed_line.str(), {20, 80}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(180, 220, 255), 2);

        const std::string cmd = gimbal.get_command_buffer();
        cv::putText(canvas, "Serial Cmd:", {20, 130}, cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(255, 255, 0), 2);
        cv::putText(canvas, cmd, {20, 170}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 0), 2);

        cv::putText(canvas, "Press SPACE to rotate | q/ESC=quit",
                    {20, 430}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(180, 180, 180), 1);

        if (target_pos.x >= 0.0f) {
            cv::circle(canvas, cv::Point(static_cast<int>(target_pos.x), static_cast<int>(target_pos.y)),
                       6, cv::Scalar(0, 0, 255), -1);
        }

        cv::drawMarker(canvas,
                       cv::Point(static_cast<int>(cam_cfg.cx), static_cast<int>(cam_cfg.cy)),
                       cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 14, 2);

        cv::imshow(main_win, canvas);

        int key = cv::waitKey(30);
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }
        if (key == ' ') {
            if (target_pos.x >= 0.0f) {
                rotating = true;
            }
        }

    }

    return 0;
}
