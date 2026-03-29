#include <opencv2/opencv.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>
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

    auto ltrim = [](std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
            return !std::isspace(ch);
        }));
    };
    auto rtrim = [](std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
            return !std::isspace(ch);
        }).base(), s.end());
    };
    ltrim(url);
    rtrim(url);
    return url;
}

// PID控制器类
class PIDController {
public:
    float kp, ki, kd;
    float integral;
    float prev_error;
    float integral_limit;
    float output_limit;
    float dt_min;
    
    PIDController(float p, float i, float d, float i_limit = 100.0f, float out_limit = 270.0f)
        : kp(p), ki(i), kd(d), integral(0), prev_error(0), 
          integral_limit(i_limit), output_limit(out_limit), dt_min(0.001f) {}
    
    float update(float error, float dt) {
        // 限制最小dt避免除零
        dt = std::max(dt, dt_min);
        
        // 比例项
        float proportional = kp * error;
        
        // 积分项（带限幅）
        integral += error * dt;
        integral = std::max(-integral_limit, std::min(integral, integral_limit));
        float integral_term = ki * integral;
        
        // 微分项
        float derivative = (error - prev_error) / dt;
        float derivative_term = kd * derivative;
        
        // 总输出
        float output = proportional + integral_term + derivative_term;
        output = std::max(-output_limit, std::min(output, output_limit));
        
        // 保存误差用于下次微分
        prev_error = error;
        
        return output;
    }
    
    void reset() {
        integral = 0;
        prev_error = 0;
    }
    
    void setGains(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
        reset();
    }
};

} // namespace

int main(int argc, char** argv) {
    int camera_index = 0;
    int width = 640;
    int height = 480;
    int input_fps = 30;
    int reconnect_ms = 1000;
    std::string stream_url;

    int serial_baud = 115200;
    std::string serial_device = "/dev/ttyACM0";
    bool enable_serial = true;

    int blue_h_min = 95;
    int blue_h_max = 135;
    int blue_s_min = 80;
    int blue_v_min = 80;
    int min_area_px = 600;

    // PID参数
    float pid_p_yaw = 0.35f;      // 偏航比例增益
    float pid_i_yaw = 0.02f;      // 偏航积分增益
    float pid_d_yaw = 0.08f;      // 偏航微分增益
    float pid_p_pitch = 0.35f;    // 俯仰比例增益
    float pid_i_pitch = 0.02f;    // 俯仰积分增益
    float pid_d_pitch = 0.08f;    // 俯仰微分增益
    float max_output_deg = 10.0f; // 单次最大调整角度（度）
    float deadzone_px = 10.0f;    // 死区（像素）
    
    // 目标角度（绝对角度）
    float yaw_angle = 270.0f;
    float pitch_angle = 60.0f;
    float yaw_zero = 270.0f;
    float pitch_zero = 60.0f;

    bool invert_yaw = false;
    bool invert_pitch = false;
    bool auto_tracking = true;     // 自动跟踪开关
    
    // 用于调试的显示选项
    bool show_pid_debug = true;

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
        } else if (arg == "--yaw" && i + 1 < argc) {
            yaw_angle = parse_float_or_default(argv[++i], yaw_angle);
        } else if (arg == "--pitch" && i + 1 < argc) {
            pitch_angle = parse_float_or_default(argv[++i], pitch_angle);
        } else if (arg == "--yaw-zero" && i + 1 < argc) {
            yaw_zero = parse_float_or_default(argv[++i], yaw_zero);
        } else if (arg == "--pitch-zero" && i + 1 < argc) {
            pitch_zero = parse_float_or_default(argv[++i], pitch_zero);
        } else if (arg == "--deadzone" && i + 1 < argc) {
            deadzone_px = parse_float_or_default(argv[++i], deadzone_px);
        } else if (arg == "--invert-yaw") {
            invert_yaw = true;
        } else if (arg == "--invert-pitch") {
            invert_pitch = true;
        } else if (arg == "--no-auto") {
            auto_tracking = false;
        }
    }

    // 参数限制
    blue_h_min = std::clamp(blue_h_min, 0, 179);
    blue_h_max = std::clamp(blue_h_max, 0, 179);
    blue_s_min = std::clamp(blue_s_min, 0, 255);
    blue_v_min = std::clamp(blue_v_min, 0, 255);
    min_area_px = std::max(min_area_px, 50);
    deadzone_px = std::max(deadzone_px, 0.0f);
    max_output_deg = std::max(max_output_deg, 0.5f);
    input_fps = std::max(input_fps, 1);
    reconnect_ms = std::max(reconnect_ms, 200);

    yaw_angle = clampf(yaw_angle, 0.0f, 270.0f);
    pitch_angle = clampf(pitch_angle, 0.0f, 270.0f);
    yaw_zero = clampf(yaw_zero, 0.0f, 270.0f);
    pitch_zero = clampf(pitch_zero, 0.0f, 270.0f);

    stream_url = normalize_stream_url(stream_url);
    if (!stream_url.empty() && (stream_url.find('<') != std::string::npos || stream_url.find('>') != std::string::npos)) {
        std::cerr << "Invalid --stream URL: " << stream_url << std::endl;
        std::cerr << "Please replace <maix_ip> with a real IP, e.g. http://192.168.1.23:8000/stream" << std::endl;
        return 1;
    }

    const bool use_stream = !stream_url.empty();
    cv::VideoCapture cap;
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
        if (use_stream) {
            std::cerr << "Failed to open stream: " << stream_url << std::endl;
        } else {
            std::cerr << "Failed to open camera index " << camera_index << std::endl;
        }
        return 1;
    }

    // 初始化云台
    GimbalControl gimbal;
    gimbal.set_yaw_zero_angle_deg(yaw_zero);
    gimbal.set_pitch_zero_angle_deg(pitch_zero);

    if (enable_serial) {
        if (!gimbal.open_serial(serial_device, serial_baud)) {
            std::cerr << "Failed to open serial: " << serial_device
                      << " @ " << serial_baud << std::endl;
        } else {
            std::cout << "Serial opened: " << serial_device
                      << " @ " << serial_baud << std::endl;
            const std::string init_cmd = "{P1500T1000P1350T1000P2300T1000P1500T1000P1500T1000}";
            bool init_ok = gimbal.send_raw_command(init_cmd);
            std::cout << "Init cmd sent: " << (init_ok ? "true" : "false") << std::endl;
            
            // 设置初始位置
            gimbal.set_yaw_angle(yaw_angle);
            gimbal.set_pitch_angle(pitch_angle);
            gimbal.send_command();
        }
    }

    // 初始化PID控制器
    PIDController pid_yaw(pid_p_yaw, pid_i_yaw, pid_d_yaw, 50.0f, max_output_deg);
    PIDController pid_pitch(pid_p_pitch, pid_i_pitch, pid_d_pitch, 50.0f, max_output_deg);
    
    // 创建显示窗口
    const std::string win_name = "Camera Gimbal Control (Blue Block)";
    cv::namedWindow(win_name, cv::WINDOW_AUTOSIZE);

    auto last_tick = std::chrono::steady_clock::now();
    float total_yaw_adjust = 0.0f;
    float total_pitch_adjust = 0.0f;
    
    // 用于性能统计
    int frame_count = 0;
    auto fps_last_time = std::chrono::steady_clock::now();

    while (true) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "Input frame read failed, reconnecting..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_ms));
            if (!open_capture()) {
                continue;
            }
            last_tick = std::chrono::steady_clock::now();
            continue;
        }

        auto now_tick = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now_tick - last_tick).count();
        last_tick = now_tick;
        
        // 限制dt范围
        dt = std::max(0.005f, std::min(dt, 0.1f));

        // 颜色检测
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

        // 计算误差（像素坐标）
        float error_x = 0.0f;
        float error_y = 0.0f;
        
        if (found_blue) {
            cv::rectangle(canvas, best_box, cv::Scalar(255, 140, 0), 2);
            cv::circle(canvas, best_center, 5, cv::Scalar(0, 0, 255), 2);
            cv::putText(canvas, "blue_target", cv::Point(best_box.x, std::max(20, best_box.y - 8)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.56, cv::Scalar(255, 140, 0), 2);

            error_x = best_center.x - static_cast<float>(center.x);
            error_y = best_center.y - static_cast<float>(center.y);
            
            // 应用死区
            float deadzone_normalized = deadzone_px;
            if (std::abs(error_x) < deadzone_normalized) error_x = 0;
            if (std::abs(error_y) < deadzone_normalized) error_y = 0;
            
            // 归一化误差：将像素误差映射到角度误差范围
            // 假设视场角(FOV)为60度，图像宽度640像素 => 每像素约0.094度
            float pixels_to_deg_x = 60.0f / width;   // 水平方向每像素对应的角度
            float pixels_to_deg_y = 45.0f / height;  // 垂直方向每像素对应的角度
            
            float angle_error_x = error_x * pixels_to_deg_x;
            float angle_error_y = error_y * pixels_to_deg_y;
            
            if (auto_tracking) {
                // PID计算需要的角度调整量（度）
                float yaw_adjust = pid_yaw.update(angle_error_x, dt);
                float pitch_adjust = pid_pitch.update(angle_error_y, dt);
                
                // 应用方向反转
                if (invert_yaw) yaw_adjust = -yaw_adjust;
                if (invert_pitch) pitch_adjust = -pitch_adjust;
                
                // 更新目标角度
                float new_yaw = yaw_angle + yaw_adjust;
                float new_pitch = pitch_angle + pitch_adjust;
                
                // 限制角度范围
                new_yaw = clampf(new_yaw, 0.0f, 270.0f);
                new_pitch = clampf(new_pitch, 0.0f, 270.0f);
                
                // 累积总调整量用于显示
                total_yaw_adjust += std::abs(yaw_adjust);
                total_pitch_adjust += std::abs(pitch_adjust);
                
                // 只有角度变化超过阈值时才发送命令
                if (std::abs(new_yaw - yaw_angle) > 0.1f || std::abs(new_pitch - pitch_angle) > 0.1f) {
                    yaw_angle = new_yaw;
                    pitch_angle = new_pitch;
                    
                    gimbal.set_yaw_angle(yaw_angle);
                    gimbal.set_pitch_angle(pitch_angle);
                    
                    if (enable_serial) {
                        gimbal.send_command();
                    }
                }
                
                // 绘制PID调试信息
                if (show_pid_debug) {
                    std::string pid_info = cv::format(
                        "PID Yaw: P=%.2f I=%.2f D=%.2f | Pitch: P=%.2f I=%.2f D=%.2f",
                        pid_yaw.kp, pid_yaw.ki, pid_yaw.kd,
                        pid_pitch.kp, pid_pitch.ki, pid_pitch.kd);
                    cv::putText(canvas, pid_info, cv::Point(16, 110),
                                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1);
                }
            }
            
            // 绘制误差线和目标指示
            cv::line(canvas, center, cv::Point(static_cast<int>(best_center.x), static_cast<int>(best_center.y)),
                     cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            
            // 绘制误差数值
            cv::putText(canvas, cv::format("Err X:%.1fpx (%.2fdeg)", error_x, error_x * 60.0f / width),
                        cv::Point(best_box.x, best_box.y + best_box.height + 15),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 0), 1);
            cv::putText(canvas, cv::format("Err Y:%.1fpx (%.2fdeg)", error_y, error_y * 45.0f / height),
                        cv::Point(best_box.x, best_box.y + best_box.height + 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 0), 1);
        } else {
            // 没有检测到目标时重置PID积分项，防止积分饱和
            pid_yaw.reset();
            pid_pitch.reset();
        }

        // 显示状态信息
        cv::putText(canvas,
                    cv::format("Target: %s | Tracking: %s", 
                               found_blue ? "YES" : "NO",
                               auto_tracking ? "ON" : "OFF"),
                    cv::Point(16, 28),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.62,
                    found_blue ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255),
                    2);

        cv::putText(canvas,
                    cv::format("Yaw:%.1f/270  Pitch:%.1f/270  Inv:%c%c", 
                               yaw_angle, pitch_angle,
                               invert_yaw ? 'Y' : 'N',
                               invert_pitch ? 'Y' : 'N'),
                    cv::Point(16, 56),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.52,
                    cv::Scalar(230, 230, 230),
                    2);

        cv::putText(canvas,
                    cv::format("HSV:[%d,%d] S>=%d V>=%d Area>=%d Deadzone:%.0fpx",
                               blue_h_min, blue_h_max, blue_s_min, blue_v_min, min_area_px, deadzone_px),
                    cv::Point(16, 84),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.45,
                    cv::Scalar(180, 220, 255),
                    1);

        cv::putText(canvas,
                    use_stream ? "Input: stream" : "Input: camera",
                    cv::Point(16, canvas.rows - 60),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.45,
                    cv::Scalar(180, 220, 255),
                    1);

        cv::putText(canvas,
                    "Q:quit  T:toggle tracking  I/K:inv pitch  J/L:inv yaw  R:reset PID",
                    cv::Point(16, canvas.rows - 35),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.45,
                    cv::Scalar(200, 200, 200),
                    1);
        
        // FPS显示
        frame_count++;
        auto now = std::chrono::steady_clock::now();
        float elapsed = std::chrono::duration<float>(now - fps_last_time).count();
        if (elapsed >= 1.0f) {
            float fps = frame_count / elapsed;
            cv::putText(canvas, cv::format("FPS:%.1f", fps),
                        cv::Point(canvas.cols - 80, 28),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1);
            frame_count = 0;
            fps_last_time = now;
        }

        cv::imshow(win_name, canvas);

        int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }
        // 控制按键
        if (key == 't' || key == 'T') {
            auto_tracking = !auto_tracking;
            std::cout << "Auto tracking: " << (auto_tracking ? "ON" : "OFF") << std::endl;
            if (!auto_tracking) {
                pid_yaw.reset();
                pid_pitch.reset();
            }
        }
        if (key == 'j' || key == 'J') {
            invert_yaw = true;
            std::cout << "Yaw invert: ON" << std::endl;
        } else if (key == 'l' || key == 'L') {
            invert_yaw = false;
            std::cout << "Yaw invert: OFF" << std::endl;
        } else if (key == 'i' || key == 'I') {
            invert_pitch = true;
            std::cout << "Pitch invert: ON" << std::endl;
        } else if (key == 'k' || key == 'K') {
            invert_pitch = false;
            std::cout << "Pitch invert: OFF" << std::endl;
        } else if (key == 'r' || key == 'R') {
            pid_yaw.reset();
            pid_pitch.reset();
            std::cout << "PID reset" << std::endl;
        }
        // PID增益实时调整（高级功能）
        else if (key == '1') {
            pid_yaw.kp += 0.05f;
            std::cout << "Yaw Kp: " << pid_yaw.kp << std::endl;
        } else if (key == '2') {
            pid_yaw.kp = std::max(0.0f, pid_yaw.kp - 0.05f);
            std::cout << "Yaw Kp: " << pid_yaw.kp << std::endl;
        } else if (key == '3') {
            pid_pitch.kp += 0.05f;
            std::cout << "Pitch Kp: " << pid_pitch.kp << std::endl;
        } else if (key == '4') {
            pid_pitch.kp = std::max(0.0f, pid_pitch.kp - 0.05f);
            std::cout << "Pitch Kp: " << pid_pitch.kp << std::endl;
        }
    }

    // 清理
    gimbal.close_serial();
    cap.release();
    cv::destroyAllWindows();
    
    std::cout << "Program terminated. Total adjustments - Yaw: " << total_yaw_adjust 
              << "deg, Pitch: " << total_pitch_adjust << "deg" << std::endl;
    
    return 0;
}