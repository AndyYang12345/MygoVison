#include <opencv2/opencv.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include "TargetTracking/GimbalControl.hpp"

namespace {

int parse_or_default(const char* text, int fallback) {
    try {
        return std::stoi(text);
    } catch (...) {
        return fallback;
    }
}

float parse_float_or_default(const char* text, float fallback) {
    try {
        return std::stof(text);
    } catch (...) {
        return fallback;
    }
}

float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(v, hi));
}

std::string normalize_stream_url(std::string url) {
    url.erase(std::remove(url.begin(), url.end(), '\\'), url.end());

    auto ltrim = [](std::string& s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
            return !std::isspace(ch);
        }));
    };
    auto rtrim = [](std::string& s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
            return !std::isspace(ch);
        }).base(), s.end());
    };

    ltrim(url);
    rtrim(url);
    return url;
}

class PIDController {
public:
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float integral_limit;
    float output_limit;
    float dt_min;

    PIDController(float p, float i, float d, float i_limit, float out_limit)
        : kp(p), ki(i), kd(d), integral(0.0f), prev_error(0.0f),
          integral_limit(i_limit), output_limit(out_limit), dt_min(0.001f) {}

    float update(float error, float dt) {
        dt = std::max(dt, dt_min);
        float proportional = kp * error;

        integral += error * dt;
        integral = std::max(-integral_limit, std::min(integral, integral_limit));
        float integral_term = ki * integral;

        float derivative = (error - prev_error) / dt;
        float derivative_term = kd * derivative;

        float output = proportional + integral_term + derivative_term;
        output = std::max(-output_limit, std::min(output, output_limit));

        prev_error = error;
        return output;
    }

    void reset() {
        integral = 0.0f;
        prev_error = 0.0f;
    }
};

} // namespace

int main(int argc, char** argv) {
    constexpr float kSafeCenterDeg = 135.0f;
    constexpr float kSafeHalfRangeDeg = 30.0f;
    constexpr float kSafeMinDeg = kSafeCenterDeg - kSafeHalfRangeDeg;
    constexpr float kSafeMaxDeg = kSafeCenterDeg + kSafeHalfRangeDeg;

    int camera_index = 0;
    int width = 640;
    int height = 480;
    int input_fps = 30;
    int reconnect_ms = 1000;
    std::string stream_url = "http://192.168.43.19:8000/stream";

    int serial_baud = 115200;
    std::string serial_device = "/dev/ttyACM0";
    bool enable_serial = true;

    int blue_h_min = 95;
    int blue_h_max = 135;
    int blue_s_min = 80;
    int blue_v_min = 80;
    int min_area_px = 600;

    float pid_p_yaw = 0.0500989f;
    float pid_i_yaw = 0.0296876f;
    float pid_d_yaw = 0.0289406f;
    float pid_p_pitch = 0.0500989f;
    float pid_i_pitch = 0.0296876f;
    float pid_d_pitch = 0.0289406f;
    float max_output_deg = 1.5f;
    float deadzone_px = 18.0f;

    float yaw_angle = kSafeCenterDeg;
    float pitch_angle = kSafeCenterDeg;
    float yaw_zero = kSafeCenterDeg;
    float pitch_zero = kSafeCenterDeg;

    bool invert_yaw = true;
    bool invert_pitch = true;
    bool auto_tracking = true;

    std::string log_path = "pixel_error_log.csv";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--camera" && i + 1 < argc) {
            camera_index = parse_or_default(argv[++i], camera_index);
        } else if (arg == "--stream" && i + 1 < argc) {
            stream_url = argv[++i];
        } else if (arg == "--width" && i + 1 < argc) {
            width = parse_or_default(argv[++i], width);
        } else if (arg == "--height" && i + 1 < argc) {
            height = parse_or_default(argv[++i], height);
        } else if (arg == "--fps" && i + 1 < argc) {
            input_fps = parse_or_default(argv[++i], input_fps);
        } else if (arg == "--reconnect-ms" && i + 1 < argc) {
            reconnect_ms = parse_or_default(argv[++i], reconnect_ms);
        } else if (arg == "--serial" && i + 1 < argc) {
            serial_device = argv[++i];
            enable_serial = true;
        } else if (arg == "--baud" && i + 1 < argc) {
            serial_baud = parse_or_default(argv[++i], serial_baud);
        } else if (arg == "--no-serial") {
            enable_serial = false;
        } else if (arg == "--h-min" && i + 1 < argc) {
            blue_h_min = parse_or_default(argv[++i], blue_h_min);
        } else if (arg == "--h-max" && i + 1 < argc) {
            blue_h_max = parse_or_default(argv[++i], blue_h_max);
        } else if (arg == "--s-min" && i + 1 < argc) {
            blue_s_min = parse_or_default(argv[++i], blue_s_min);
        } else if (arg == "--v-min" && i + 1 < argc) {
            blue_v_min = parse_or_default(argv[++i], blue_v_min);
        } else if (arg == "--min-area" && i + 1 < argc) {
            min_area_px = parse_or_default(argv[++i], min_area_px);
        } else if (arg == "--pid-p-yaw" && i + 1 < argc) {
            pid_p_yaw = parse_float_or_default(argv[++i], pid_p_yaw);
        } else if (arg == "--pid-i-yaw" && i + 1 < argc) {
            pid_i_yaw = parse_float_or_default(argv[++i], pid_i_yaw);
        } else if (arg == "--pid-d-yaw" && i + 1 < argc) {
            pid_d_yaw = parse_float_or_default(argv[++i], pid_d_yaw);
        } else if (arg == "--pid-p-pitch" && i + 1 < argc) {
            pid_p_pitch = parse_float_or_default(argv[++i], pid_p_pitch);
        } else if (arg == "--pid-i-pitch" && i + 1 < argc) {
            pid_i_pitch = parse_float_or_default(argv[++i], pid_i_pitch);
        } else if (arg == "--pid-d-pitch" && i + 1 < argc) {
            pid_d_pitch = parse_float_or_default(argv[++i], pid_d_pitch);
        } else if (arg == "--max-output" && i + 1 < argc) {
            max_output_deg = parse_float_or_default(argv[++i], max_output_deg);
        } else if (arg == "--deadzone" && i + 1 < argc) {
            deadzone_px = parse_float_or_default(argv[++i], deadzone_px);
        } else if (arg == "--yaw" && i + 1 < argc) {
            yaw_angle = parse_float_or_default(argv[++i], yaw_angle);
        } else if (arg == "--pitch" && i + 1 < argc) {
            pitch_angle = parse_float_or_default(argv[++i], pitch_angle);
        } else if (arg == "--yaw-zero" && i + 1 < argc) {
            yaw_zero = parse_float_or_default(argv[++i], yaw_zero);
        } else if (arg == "--pitch-zero" && i + 1 < argc) {
            pitch_zero = parse_float_or_default(argv[++i], pitch_zero);
        } else if (arg == "--invert-yaw") {
            invert_yaw = true;
        } else if (arg == "--invert-pitch") {
            invert_pitch = true;
        } else if (arg == "--no-auto") {
            auto_tracking = false;
        } else if (arg == "--log" && i + 1 < argc) {
            log_path = argv[++i];
        }
    }

    blue_h_min = std::clamp(blue_h_min, 0, 179);
    blue_h_max = std::clamp(blue_h_max, 0, 179);
    blue_s_min = std::clamp(blue_s_min, 0, 255);
    blue_v_min = std::clamp(blue_v_min, 0, 255);
    min_area_px = std::max(min_area_px, 50);
    deadzone_px = std::max(deadzone_px, 0.0f);
    max_output_deg = std::max(max_output_deg, 0.5f);
    input_fps = std::max(input_fps, 1);
    reconnect_ms = std::max(reconnect_ms, 200);

    yaw_angle = clampf(yaw_angle, kSafeMinDeg, kSafeMaxDeg);
    pitch_angle = clampf(pitch_angle, kSafeMinDeg, kSafeMaxDeg);
    yaw_zero = clampf(yaw_zero, kSafeMinDeg, kSafeMaxDeg);
    pitch_zero = clampf(pitch_zero, kSafeMinDeg, kSafeMaxDeg);

    stream_url = normalize_stream_url(stream_url);

    std::ofstream log_file(log_path, std::ios::out);
    if (!log_file.is_open()) {
        std::cerr << "Failed to open log file: " << log_path << std::endl;
        return 1;
    }

    log_file << "timestamp_ms,found_target,center_x,center_y,target_x,target_y,error_x,error_y,"
             << "filtered_error_x,filtered_error_y,yaw_angle,pitch_angle,yaw_adjust,pitch_adjust,auto_tracking\n";

    cv::VideoCapture cap;
    const bool use_stream = !stream_url.empty();
    auto open_capture = [&]() -> bool {
        if (cap.isOpened()) {
            cap.release();
        }

        bool ok = false;
        if (use_stream) {
            ok = cap.open(stream_url, cv::CAP_FFMPEG);
            if (!ok) {
                ok = cap.open(stream_url);
            }
        } else {
            ok = cap.open(camera_index);
            if (ok) {
                cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
                cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
                cap.set(cv::CAP_PROP_FPS, input_fps);
            }
        }
        return ok;
    };

    if (!open_capture()) {
        std::cerr << (use_stream ? "Failed to open stream: " : "Failed to open camera index ")
                  << (use_stream ? stream_url : std::to_string(camera_index)) << std::endl;
        return 1;
    }

    GimbalControl gimbal;
    gimbal.set_yaw_zero_angle_deg(yaw_zero);
    gimbal.set_pitch_zero_angle_deg(pitch_zero);

    if (enable_serial && gimbal.open_serial(serial_device, serial_baud)) {
        gimbal.set_yaw_angle(yaw_angle);
        gimbal.set_pitch_angle(pitch_angle);
        gimbal.get_command();
        gimbal.send_command();
    }

    PIDController pid_yaw(pid_p_yaw, pid_i_yaw, pid_d_yaw, 50.0f, max_output_deg);
    PIDController pid_pitch(pid_p_pitch, pid_i_pitch, pid_d_pitch, 50.0f, max_output_deg);

    const float error_lpf_alpha = 0.18f;
    float filtered_error_x = 0.0f;
    float filtered_error_y = 0.0f;
    bool filter_initialized = false;

    const auto min_send_interval = std::chrono::milliseconds(50);
    auto last_send_time = std::chrono::steady_clock::now() - min_send_interval;

    const auto start_time = std::chrono::steady_clock::now();
    auto last_tick = start_time;

    const std::string win_name = "Gimbal Error Logger";
    cv::namedWindow(win_name, cv::WINDOW_AUTOSIZE);

    while (true) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_ms));
            if (!open_capture()) {
                continue;
            }
            last_tick = std::chrono::steady_clock::now();
            continue;
        }

        const auto now_tick = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now_tick - last_tick).count();
        last_tick = now_tick;
        dt = std::max(0.005f, std::min(dt, 0.1f));

        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        cv::Mat mask;
        cv::inRange(
            hsv,
            cv::Scalar(blue_h_min, blue_s_min, blue_v_min),
            cv::Scalar(blue_h_max, 255, 255),
            mask);

        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::GaussianBlur(mask, mask, cv::Size(7, 7), 0.0);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        const cv::Point center(frame.cols / 2, frame.rows / 2);
        cv::Mat canvas = frame.clone();
        cv::drawMarker(canvas, center, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 18, 2);

        bool found_blue = false;
        cv::Rect best_box;
        cv::Point2f best_center(0.0f, 0.0f);
        double best_area = 0.0;

        for (const auto& contour : contours) {
            const double area = cv::contourArea(contour);
            if (area < static_cast<double>(min_area_px) || area <= best_area) {
                continue;
            }
            cv::Moments m = cv::moments(contour);
            if (std::abs(m.m00) < 1e-5) {
                continue;
            }
            best_area = area;
            best_box = cv::boundingRect(contour);
            best_center = cv::Point2f(
                static_cast<float>(m.m10 / m.m00),
                static_cast<float>(m.m01 / m.m00));
            found_blue = true;
        }

        float error_x = 0.0f;
        float error_y = 0.0f;
        float yaw_adjust = 0.0f;
        float pitch_adjust = 0.0f;

        if (found_blue) {
            cv::rectangle(canvas, best_box, cv::Scalar(255, 140, 0), 2);
            cv::circle(canvas, best_center, 5, cv::Scalar(0, 0, 255), 2);

            error_x = best_center.x - static_cast<float>(center.x);
            error_y = best_center.y - static_cast<float>(center.y);

            if (std::abs(error_x) < deadzone_px) error_x = 0.0f;
            if (std::abs(error_y) < deadzone_px) error_y = 0.0f;

            if (!filter_initialized) {
                filtered_error_x = error_x;
                filtered_error_y = error_y;
                filter_initialized = true;
            } else {
                filtered_error_x = (1.0f - error_lpf_alpha) * filtered_error_x + error_lpf_alpha * error_x;
                filtered_error_y = (1.0f - error_lpf_alpha) * filtered_error_y + error_lpf_alpha * error_y;
            }

            if (auto_tracking) {
                yaw_adjust = pid_yaw.update(filtered_error_x, dt);
                pitch_adjust = pid_pitch.update(filtered_error_y, dt);

                const float near_center_px = deadzone_px * 3.0f;
                const float err_norm_x = std::min(1.0f, std::abs(filtered_error_x) / std::max(near_center_px, 1.0f));
                const float err_norm_y = std::min(1.0f, std::abs(filtered_error_y) / std::max(near_center_px, 1.0f));
                yaw_adjust *= (0.25f + 0.75f * err_norm_x);
                pitch_adjust *= (0.25f + 0.75f * err_norm_y);

                if (invert_yaw) yaw_adjust = -yaw_adjust;
                if (invert_pitch) pitch_adjust = -pitch_adjust;

                float new_yaw = clampf(yaw_angle + yaw_adjust, kSafeMinDeg, kSafeMaxDeg);
                float new_pitch = clampf(pitch_angle + pitch_adjust, kSafeMinDeg, kSafeMaxDeg);

                if (std::abs(new_yaw - yaw_angle) > 0.1f || std::abs(new_pitch - pitch_angle) > 0.1f) {
                    yaw_angle = new_yaw;
                    pitch_angle = new_pitch;

                    gimbal.set_yaw_angle(yaw_angle);
                    gimbal.set_pitch_angle(pitch_angle);

                    if (enable_serial && (now_tick - last_send_time) >= min_send_interval) {
                        gimbal.get_command();
                        gimbal.send_command();
                        last_send_time = now_tick;
                    }
                }
            }
        } else {
            pid_yaw.reset();
            pid_pitch.reset();
            filter_initialized = false;
            filtered_error_x = 0.0f;
            filtered_error_y = 0.0f;
        }

        const long long ts_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_tick - start_time).count();
        const float target_x = found_blue ? best_center.x : -1.0f;
        const float target_y = found_blue ? best_center.y : -1.0f;

        log_file << ts_ms << ","
                 << (found_blue ? 1 : 0) << ","
                 << center.x << ","
                 << center.y << ","
                 << target_x << ","
                 << target_y << ","
                 << error_x << ","
                 << error_y << ","
                 << filtered_error_x << ","
                 << filtered_error_y << ","
                 << yaw_angle << ","
                 << pitch_angle << ","
                 << yaw_adjust << ","
                 << pitch_adjust << ","
                 << (auto_tracking ? 1 : 0) << "\n";

        if ((ts_ms % 1000) < 15) {
            log_file.flush();
        }

        cv::putText(canvas,
                    cv::format("ErrX:%.1f  ErrY:%.1f  FilX:%.1f  FilY:%.1f", error_x, error_y, filtered_error_x, filtered_error_y),
                    cv::Point(16, 28),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.55,
                    cv::Scalar(0, 255, 255),
                    2);

        cv::putText(canvas,
                    cv::format("Yaw:%.1f Pitch:%.1f Log:%s", yaw_angle, pitch_angle, log_path.c_str()),
                    cv::Point(16, 56),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(200, 220, 255),
                    1);

        cv::putText(canvas,
                    "Q:quit  T:auto  I/K:pitch inv  J/L:yaw inv",
                    cv::Point(16, canvas.rows - 24),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.45,
                    cv::Scalar(200, 200, 200),
                    1);

        cv::imshow(win_name, canvas);

        const int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27) {
            break;
        }
        if (key == 't' || key == 'T') {
            auto_tracking = !auto_tracking;
            if (!auto_tracking) {
                pid_yaw.reset();
                pid_pitch.reset();
            }
        } else if (key == 'j' || key == 'J') {
            invert_yaw = true;
        } else if (key == 'l' || key == 'L') {
            invert_yaw = false;
        } else if (key == 'i' || key == 'I') {
            invert_pitch = true;
        } else if (key == 'k' || key == 'K') {
            invert_pitch = false;
        }
    }

    log_file.flush();
    log_file.close();
    gimbal.close_serial();
    cap.release();
    cv::destroyAllWindows();

    return 0;
}
