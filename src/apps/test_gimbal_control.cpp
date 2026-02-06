#include <opencv2/opencv.hpp>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>

#include "TargetSim/Target3DGenerator.hpp"
#include "TargetTracking/GimbalControl.hpp"

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
    const int height = 480;
    Target3DGenerator generator(width, height, 30.0f);

    SimulationCamera::CameraIntrinsics intr;
    intr.fx = width * 0.9f;
    intr.fy = width * 0.9f;
    intr.cx = width * 0.5f;
    intr.cy = height * 0.5f;
    generator.set_camera_intrinsics(intr);

    SimulationCamera::CameraPose pose;
    pose.position = cv::Point3f(0.0f, 0.0f, -600.0f);
    pose.rotation = cv::Point3f(0.0f, 0.0f, 0.0f);
    generator.set_camera_pose(pose);
    generator.set_target_plane(cv::Point3f(0.0f, 0.0f, 1.0f), 600.0f);
    generator.set_random_interval(0.6f);
    generator.generator().pause();

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
    bool request_new_target = true;

    std::vector<cv::Point> aim_trail;
    std::vector<cv::Point> target_trail;
    const size_t max_trail = 300;

    auto last_tick = std::chrono::steady_clock::now();

    while (true) {
        auto now_tick = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now_tick - last_tick).count();
        last_tick = now_tick;
        dt = clamp_value(dt, 0.001f, 0.05f);

        cv::Mat canvas(480, 720, CV_8UC3, cv::Scalar(20, 20, 20));

        if (request_new_target) {
            float current_time = generator.generator().get_current_time();
            generator.generator().set_current_time(current_time + 0.61f);
            request_new_target = false;
        }
        auto result = generator.generate_projected_frame();

        cv::Point2f target_pos(-1.0f, -1.0f);
        if (!result.projected_positions.empty()) {
            target_pos = result.projected_positions[0];
        }

        if (target_pos.x >= 0.0f) {
            float dx = target_pos.x - intr.cx;
            float dy = target_pos.y - intr.cy;
            float target_yaw_offset = std::atan2(dx, intr.fx) * 180.0f / static_cast<float>(CV_PI);
            float target_pitch_offset = -std::atan2(dy, intr.fy) * 180.0f / static_cast<float>(CV_PI);

            float target_pitch = 90.0f + target_pitch_offset;
            float target_yaw = 135.0f + target_yaw_offset;

            float pitch_error = target_pitch - pitch_angle;
            float yaw_error = target_yaw - yaw_angle;

            float pitch_speed_cmd = pid_step(pitch_error, dt, pid_pitch, integral_limit);
            float yaw_speed_cmd = pid_step(yaw_error, dt, pid_yaw, integral_limit);

            pitch_speed = clamp_value(pitch_speed_cmd, -max_speed, max_speed);
            yaw_speed = clamp_value(yaw_speed_cmd, -max_speed, max_speed);

            pitch_angle += pitch_speed * dt;
            yaw_angle += yaw_speed * dt;

            if (std::abs(pitch_error) < settle_threshold && std::abs(yaw_error) < settle_threshold) {
                settle_timer += dt;
                if (settle_timer >= settle_hold) {
                    request_new_target = true;
                    settle_timer = 0.0f;
                    pid_pitch = {};
                    pid_yaw = {};
                    aim_trail.clear();
                    target_trail.clear();
                }
            } else {
                settle_timer = 0.0f;
            }
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
                    cv::Scalar(255, 200, 120), 2);
        cv::putText(canvas, cmd, {20, 170}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(255, 255, 255), 2);

        cv::putText(canvas, "PID tracking random target | q/ESC=quit",
                    {20, 430}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(180, 180, 180), 1);

        if (target_pos.x >= 0.0f) {
            cv::Point target_point(static_cast<int>(target_pos.x), static_cast<int>(target_pos.y));
            target_trail.push_back(target_point);
            if (target_trail.size() > max_trail) {
                target_trail.erase(target_trail.begin());
            }
            for (size_t i = 1; i < target_trail.size(); ++i) {
                cv::line(canvas, target_trail[i - 1], target_trail[i], cv::Scalar(0, 0, 180), 1);
            }
            cv::circle(canvas, target_point, 6, cv::Scalar(0, 0, 255), -1);
        }

        float yaw_rad = (yaw_angle - 135.0f) * static_cast<float>(CV_PI) / 180.0f;
        float pitch_rad = (pitch_angle - 90.0f) * static_cast<float>(CV_PI) / 180.0f;
        float aim_x = intr.cx + std::tan(yaw_rad) * intr.fx;
        float aim_y = intr.cy - std::tan(pitch_rad) * intr.fy;
        cv::Point aim_point(static_cast<int>(aim_x), static_cast<int>(aim_y));
        aim_trail.push_back(aim_point);
        if (aim_trail.size() > max_trail) {
            aim_trail.erase(aim_trail.begin());
        }
        for (size_t i = 1; i < aim_trail.size(); ++i) {
            cv::line(canvas, aim_trail[i - 1], aim_trail[i], cv::Scalar(0, 200, 0), 1);
        }
        cv::drawMarker(canvas, aim_point, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 14, 2);

        cv::imshow(main_win, canvas);

        int key = cv::waitKey(30);
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }

    }

    return 0;
}
