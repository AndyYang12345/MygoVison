#include <opencv2/opencv.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <iostream>
#include <iomanip>
#include <string>
#include <thread>
#include <random>

#include "TargetTracking/GimbalControl.hpp"
#include "TargetTracking/GeneticAlgorithm.hpp"

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

    // PID参数（按角度误差控制，输出为角速度 deg/s）
//     2.40262,0,0.207062
    float pid_p_yaw = 2.40262f;
    float pid_i_yaw = 0.05f;
    float pid_d_yaw = 0.307062f;
    float pid_p_pitch = 2.40262f;
    float pid_i_pitch = 0.05f;
    float pid_d_pitch = 0.307062f;
    float max_output_deg = 150.0f;  // 单次最大角速度（度/秒）
    float deadzone_px = 8.0f;    // 死区（像素）
    
    // 目标角度（绝对角度）
    float yaw_angle = kSafeCenterDeg;
    float pitch_angle = kSafeCenterDeg;
    float yaw_zero = kSafeCenterDeg;
    float pitch_zero = kSafeCenterDeg;

    bool invert_yaw = false;
    bool invert_pitch = false;
    bool auto_tracking = true;     // 自动跟踪开关

    // 在线GA参数
    bool ga_online_enabled = false;
    int ga_population_size = 10;
    float ga_eval_seconds = 10.0f;
    float ga_mutation_sigma = 0.010f;
    std::string ga_log_path = "ga_best_pid_log.csv";

    // 固定PID日志（默认开启，仅在GA关闭时记录）
    bool fixed_pid_log_enabled = true;
    std::string fixed_pid_log_path = "fixed_pid_distance_log.csv";
    
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
        } else if (arg == "--ga-online") {
            ga_online_enabled = true;
        } else if (arg == "--no-ga-online") {
            ga_online_enabled = false;
        } else if (arg == "--ga-pop" && i + 1 < argc) {
            ga_population_size = parse_or_default(argv[++i], ga_population_size);
        } else if (arg == "--ga-test-sec" && i + 1 < argc) {
            ga_eval_seconds = parse_float_or_default(argv[++i], ga_eval_seconds);
        } else if (arg == "--ga-mutation-sigma" && i + 1 < argc) {
            ga_mutation_sigma = parse_float_or_default(argv[++i], ga_mutation_sigma);
        } else if (arg == "--ga-log" && i + 1 < argc) {
            ga_log_path = argv[++i];
        } else if (arg == "--fixed-log" && i + 1 < argc) {
            fixed_pid_log_path = argv[++i];
        } else if (arg == "--no-fixed-log") {
            fixed_pid_log_enabled = false;
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
    ga_population_size = std::max(2, ga_population_size);
    ga_eval_seconds = std::max(2.0f, ga_eval_seconds);
    ga_mutation_sigma = std::max(0.001f, ga_mutation_sigma);

    yaw_angle = clampf(yaw_angle, kSafeMinDeg, kSafeMaxDeg);
    pitch_angle = clampf(pitch_angle, kSafeMinDeg, kSafeMaxDeg);
    yaw_zero = clampf(yaw_zero, kSafeMinDeg, kSafeMaxDeg);
    pitch_zero = clampf(pitch_zero, kSafeMinDeg, kSafeMaxDeg);

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
            gimbal.get_command();
            gimbal.send_command();
        }
    }

    // 初始化PID控制器
    PIDController pid_yaw(pid_p_yaw, pid_i_yaw, pid_d_yaw, 20.0f, max_output_deg);
    PIDController pid_pitch(pid_p_pitch, pid_i_pitch, pid_d_pitch, 20.0f, max_output_deg);

    struct OnlineGaIndividual {
        Genome genome;
        float fitness = -std::numeric_limits<float>::infinity();
    };
    struct OnlineGaMetrics {
        float err_integral = 0.0f;
        float err_square_integral = 0.0f;
        float lost_time = 0.0f;
        float settle_time = 0.0f;
        int oscillation_count = 0;
        bool settled = false;
        bool has_prev = false;
        float prev_error_x = 0.0f;
        float prev_error_y = 0.0f;
    };
    enum class OnlineGaPhase {
        EvaluateGeneration,
        RunBestAndWaitEnter
    };

    auto sign_changed = [](float a, float b) {
        return (a > 0.0f && b < 0.0f) || (a < 0.0f && b > 0.0f);
    };

    std::random_device rd;
    std::mt19937 ga_rng(rd());
    Genome ga_seed;
    ga_seed.p = pid_p_yaw;
    ga_seed.i = pid_i_yaw;
    ga_seed.d = pid_d_yaw;
    ga_seed.p_min = 0.0f;
    ga_seed.p_max = 80.0f;
    ga_seed.i_min = 0.0f;
    ga_seed.i_max = 5.0f;
    ga_seed.d_min = 0.0f;
    ga_seed.d_max = 2.0f;
    ga_seed.clamp();

    int ga_generation = 0;
    int ga_candidate_index = 0;
    OnlineGaPhase ga_phase = OnlineGaPhase::EvaluateGeneration;
    std::vector<OnlineGaIndividual> ga_population;
    OnlineGaMetrics ga_metrics;

    auto apply_genome = [&](const Genome& g) {
        pid_yaw.setGains(g.p, g.i, g.d);
        pid_pitch.setGains(g.p, g.i, g.d);
    };

    auto reset_pose_to_center = [&]() {
        yaw_angle = kSafeCenterDeg;
        pitch_angle = kSafeCenterDeg;
        gimbal.set_yaw_angle(yaw_angle);
        gimbal.set_pitch_angle(pitch_angle);
        if (enable_serial) {
            gimbal.get_command();
            gimbal.send_command();
        }
    };

    auto build_generation = [&](const Genome& seed) {
        ga_population.clear();
        ga_population.reserve(ga_population_size);

        OnlineGaIndividual elite;
        elite.genome = seed;
        elite.genome.clamp();
        ga_population.push_back(elite);

        for (int i = 1; i < ga_population_size; ++i) {
            Genome parent_a = seed;
            Genome parent_b = seed;
            parent_a.mutate(ga_rng, 1.0f, ga_mutation_sigma);
            parent_b.mutate(ga_rng, 1.0f, ga_mutation_sigma);
            Genome child = Genome::crossover(parent_a, parent_b, ga_rng);
            child.mutate(ga_rng, 1.0f, ga_mutation_sigma * 0.7f);
            child.clamp();

            OnlineGaIndividual ind;
            ind.genome = child;
            ga_population.push_back(ind);
        }
    };

    auto reset_ga_metrics = [&]() {
        ga_metrics = OnlineGaMetrics{};
    };

    std::ifstream ga_log_check(ga_log_path);
    const bool ga_log_exists = ga_log_check.good();
    ga_log_check.close();
    if (!ga_log_exists) {
        std::ofstream ga_log_init(ga_log_path, std::ios::out);
        ga_log_init << "generation,best_index,fitness,p,i,d\n";
    }

    std::ofstream fixed_pid_log;
    if (fixed_pid_log_enabled && !ga_online_enabled) {
        fixed_pid_log.open(fixed_pid_log_path, std::ios::out | std::ios::trunc);
        if (!fixed_pid_log.is_open()) {
            std::cerr << "Failed to open fixed PID log file: " << fixed_pid_log_path << std::endl;
        } else {
            fixed_pid_log << "timestamp_ms,found_target,target_x,target_y,center_x,center_y,distance_px,"
                          << "filtered_error_x,filtered_error_y,yaw_angle,pitch_angle,kp_yaw,ki_yaw,kd_yaw,"
                          << "kp_pitch,ki_pitch,kd_pitch\n";
            std::cout << "Fixed PID log file: " << fixed_pid_log_path << std::endl;
        }
    }

    auto ga_eval_start = std::chrono::steady_clock::now();
    if (ga_online_enabled) {
        build_generation(ga_seed);
        if (!ga_population.empty()) {
            apply_genome(ga_population.front().genome);
        }
    }
    
    // 抗抖参数
    const float error_lpf_alpha = 0.55f;
    float filtered_error_x = 0.0f;
    float filtered_error_y = 0.0f;
    bool filter_initialized = false;
    
    // 目标中心卡尔曼滤波（状态: x, y, vx, vy）
    cv::KalmanFilter target_kf(4, 2, 0, CV_32F);
    target_kf.measurementMatrix = cv::Mat::zeros(2, 4, CV_32F);
    target_kf.measurementMatrix.at<float>(0, 0) = 1.0f;
    target_kf.measurementMatrix.at<float>(1, 1) = 1.0f;
    target_kf.transitionMatrix = cv::Mat::eye(4, 4, CV_32F);
    target_kf.processNoiseCov = cv::Mat::eye(4, 4, CV_32F) * 0.2f;
    target_kf.measurementNoiseCov = cv::Mat::eye(2, 2, CV_32F) * 1.0f;
    target_kf.errorCovPost = cv::Mat::eye(4, 4, CV_32F);
    bool kf_initialized = false;
    int kf_lost_frames = 0;
    const int kf_max_predict_frames = 4;

    const auto min_send_interval = std::chrono::milliseconds(25);
    auto last_send_time = std::chrono::steady_clock::now() - min_send_interval;

    // 创建显示窗口
    const std::string win_name = "Camera Gimbal Control (Blue Block)";
    bool gui_enabled = true;
    try {
        cv::namedWindow(win_name, cv::WINDOW_AUTOSIZE);
    } catch (const cv::Exception& e) {
        gui_enabled = false;
        std::cerr << "OpenCV GUI init failed, running headless: " << e.what() << std::endl;
    }

    auto last_tick = std::chrono::steady_clock::now();
    const auto run_start_tick = last_tick;
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

        target_kf.transitionMatrix.at<float>(0, 2) = dt;
        target_kf.transitionMatrix.at<float>(1, 3) = dt;
        cv::Mat kf_pred = target_kf.predict();
        const cv::Point2f pred_center(
            kf_pred.at<float>(0),
            kf_pred.at<float>(1));

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
        float distance_px = -1.0f;
        bool has_control_target = false;
        cv::Point2f control_center(0.0f, 0.0f);
        bool using_predicted_target = false;
        
        if (found_blue) {
            cv::rectangle(canvas, best_box, cv::Scalar(255, 140, 0), 2);
            cv::circle(canvas, best_center, 5, cv::Scalar(0, 0, 255), 2);
            cv::putText(canvas, "blue_target", cv::Point(best_box.x, std::max(20, best_box.y - 8)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.56, cv::Scalar(255, 140, 0), 2);

            if (!kf_initialized) {
                target_kf.statePost.at<float>(0) = best_center.x;
                target_kf.statePost.at<float>(1) = best_center.y;
                target_kf.statePost.at<float>(2) = 0.0f;
                target_kf.statePost.at<float>(3) = 0.0f;
                kf_initialized = true;
            }

            cv::Mat measurement(2, 1, CV_32F);
            measurement.at<float>(0) = best_center.x;
            measurement.at<float>(1) = best_center.y;
            target_kf.correct(measurement);
            // 优先使用实时检测值做控制，避免“预测+校正”在闭环里引入切向漂移
            control_center = best_center;
            has_control_target = true;
            kf_lost_frames = 0;
        } else if (kf_initialized && kf_lost_frames < kf_max_predict_frames) {
            control_center = pred_center;
            has_control_target = true;
            using_predicted_target = true;
            kf_lost_frames++;
        }

        if (has_control_target) {
            error_x = control_center.x - static_cast<float>(center.x);
            error_y = control_center.y - static_cast<float>(center.y);
            distance_px = std::sqrt(error_x * error_x + error_y * error_y);

            cv::circle(canvas,
                       cv::Point(static_cast<int>(control_center.x), static_cast<int>(control_center.y)),
                       4,
                       using_predicted_target ? cv::Scalar(0, 165, 255) : cv::Scalar(255, 255, 0),
                       2);
            if (using_predicted_target) {
                cv::putText(canvas,
                            "pred_target",
                            cv::Point(static_cast<int>(control_center.x) + 8, static_cast<int>(control_center.y) - 6),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.45,
                            cv::Scalar(0, 165, 255),
                            1);
            }
            
            // 应用像素死区
            if (std::abs(error_x) < deadzone_px) error_x = 0;
            if (std::abs(error_y) < deadzone_px) error_y = 0;

            // 误差低通滤波，检测有效时平滑；仅预测时不再叠加滤波，降低相位滞后
            if (!filter_initialized) {
                filtered_error_x = error_x;
                filtered_error_y = error_y;
                filter_initialized = true;
            } else if (using_predicted_target) {
                filtered_error_x = error_x;
                filtered_error_y = error_y;
            } else {
                filtered_error_x = (1.0f - error_lpf_alpha) * filtered_error_x + error_lpf_alpha * error_x;
                filtered_error_y = (1.0f - error_lpf_alpha) * filtered_error_y + error_lpf_alpha * error_y;
            }

            if (auto_tracking) {
                const float fx = std::max(1.0f, static_cast<float>(frame.cols) * 0.6f);
                const float fy = std::max(1.0f, static_cast<float>(frame.rows) * 0.6f);
                const float px_to_deg = 180.0f / static_cast<float>(CV_PI);

                float yaw_error_deg = -std::atan2(filtered_error_x, fx) * px_to_deg;
                float pitch_error_deg = -std::atan2(filtered_error_y, fy) * px_to_deg;

                if (invert_yaw) yaw_error_deg = -yaw_error_deg;
                if (invert_pitch) pitch_error_deg = -pitch_error_deg;

                // 改成与 TargetTrackingPipeline 一致的速度型 PID：输出角速度，再按 dt 积分到角度
                float yaw_speed_cmd = pid_yaw.update(yaw_error_deg, dt);
                float pitch_speed_cmd = pid_pitch.update(pitch_error_deg, dt);

                float yaw_adjust = yaw_speed_cmd * dt;
                float pitch_adjust = pitch_speed_cmd * dt;

                float new_yaw = clampf(yaw_angle + yaw_adjust, kSafeMinDeg, kSafeMaxDeg);
                float new_pitch = clampf(pitch_angle + pitch_adjust, kSafeMinDeg, kSafeMaxDeg);

                total_yaw_adjust += std::abs(new_yaw - yaw_angle);
                total_pitch_adjust += std::abs(new_pitch - pitch_angle);

                yaw_angle = new_yaw;
                pitch_angle = new_pitch;

                gimbal.set_yaw_speed(std::abs(yaw_speed_cmd));
                gimbal.set_pitch_speed(std::abs(pitch_speed_cmd));
                gimbal.set_yaw_angle(yaw_angle);
                gimbal.set_pitch_angle(pitch_angle);

                if (enable_serial && (now_tick - last_send_time) >= min_send_interval) {
                    gimbal.get_command();
                    gimbal.send_command();
                    last_send_time = now_tick;
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
            cv::line(canvas, center, cv::Point(static_cast<int>(control_center.x), static_cast<int>(control_center.y)),
                     cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            
            // 绘制误差数值
                cv::putText(canvas, cv::format("Err X:%.1fpx", error_x),
                        cv::Point(best_box.x, best_box.y + best_box.height + 15),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 0), 1);
                cv::putText(canvas, cv::format("Err Y:%.1fpx", error_y),
                        cv::Point(best_box.x, best_box.y + best_box.height + 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 0), 1);
        } else {
            // 没有检测到目标时重置PID积分项，防止积分饱和
            pid_yaw.reset();
            pid_pitch.reset();
            filter_initialized = false;
        }

        if (ga_online_enabled) {
            auto_tracking = true;

            const float elapsed_eval =
                std::chrono::duration<float>(now_tick - ga_eval_start).count();

            if (ga_phase == OnlineGaPhase::EvaluateGeneration &&
                ga_candidate_index < static_cast<int>(ga_population.size())) {
                if (!found_blue) {
                    ga_metrics.lost_time += dt;
                } else {
                    const float err_mag = std::sqrt(
                        filtered_error_x * filtered_error_x +
                        filtered_error_y * filtered_error_y);

                    if (err_mag > deadzone_px) {
                        ga_metrics.err_integral += err_mag * dt;
                        ga_metrics.err_square_integral += err_mag * err_mag * dt;

                        if (ga_metrics.has_prev) {
                            if (std::abs(filtered_error_x) > deadzone_px &&
                                sign_changed(filtered_error_x, ga_metrics.prev_error_x)) {
                                ga_metrics.oscillation_count++;
                            }
                            if (std::abs(filtered_error_y) > deadzone_px &&
                                sign_changed(filtered_error_y, ga_metrics.prev_error_y)) {
                                ga_metrics.oscillation_count++;
                            }
                        }
                    } else if (!ga_metrics.settled) {
                        ga_metrics.settled = true;
                        ga_metrics.settle_time = elapsed_eval;
                    }

                    ga_metrics.prev_error_x = filtered_error_x;
                    ga_metrics.prev_error_y = filtered_error_y;
                    ga_metrics.has_prev = true;
                }

                if (elapsed_eval >= ga_eval_seconds) {
                    const float settle_term = ga_metrics.settled ? ga_metrics.settle_time : ga_eval_seconds;
                    const float convergence_term = settle_term / std::max(ga_eval_seconds, 0.001f);
                    const float cost =
                        1.0f * ga_metrics.err_integral +
                        0.01f * ga_metrics.err_square_integral +
                        2.5f * ga_metrics.lost_time +
                        6.0f * convergence_term +
                        0.15f * static_cast<float>(ga_metrics.oscillation_count);
                    ga_population[ga_candidate_index].fitness = -cost;

                    ga_candidate_index++;
                    if (ga_candidate_index < static_cast<int>(ga_population.size())) {
                        apply_genome(ga_population[ga_candidate_index].genome);
                        pid_yaw.output_limit = max_output_deg;
                        pid_pitch.output_limit = max_output_deg;
                        filter_initialized = false;
                        reset_ga_metrics();
                        reset_pose_to_center();
                        ga_eval_start = now_tick;
                    } else {
                        int best_idx = 0;
                        float best_fit = ga_population[0].fitness;
                        for (int i = 1; i < static_cast<int>(ga_population.size()); ++i) {
                            if (ga_population[i].fitness > best_fit) {
                                best_fit = ga_population[i].fitness;
                                best_idx = i;
                            }
                        }

                        const Genome best = ga_population[best_idx].genome;
                        apply_genome(best);
                        pid_yaw.output_limit = max_output_deg;
                        pid_pitch.output_limit = max_output_deg;
                        reset_pose_to_center();
                        ga_seed = best;

                        std::ofstream ga_log(ga_log_path, std::ios::app);
                        ga_log << ga_generation << ","
                               << best_idx << ","
                               << best_fit << ","
                               << best.p << ","
                               << best.i << ","
                               << best.d << "\n";

                        std::cout << "[GA] Generation " << ga_generation
                                  << " done. Best idx=" << best_idx
                                  << " fitness=" << best_fit
                                  << " PID(" << best.p << ", " << best.i << ", " << best.d << ")"
                                  << std::endl;

                        ga_phase = OnlineGaPhase::RunBestAndWaitEnter;
                        reset_ga_metrics();
                    }
                }
            }
        }

        if (fixed_pid_log.is_open()) {
            const long long ts_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_tick - run_start_tick).count();
            const float target_x = found_blue ? best_center.x : -1.0f;
            const float target_y = found_blue ? best_center.y : -1.0f;

            fixed_pid_log << ts_ms << ","
                          << (found_blue ? 1 : 0) << ","
                          << target_x << ","
                          << target_y << ","
                          << center.x << ","
                          << center.y << ","
                          << distance_px << ","
                          << filtered_error_x << ","
                          << filtered_error_y << ","
                          << yaw_angle << ","
                          << pitch_angle << ","
                          << pid_yaw.kp << ","
                          << pid_yaw.ki << ","
                          << pid_yaw.kd << ","
                          << pid_pitch.kp << ","
                          << pid_pitch.ki << ","
                          << pid_pitch.kd << "\n";

            if ((ts_ms % 1000) < 15) {
                fixed_pid_log.flush();
            }
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
                    cv::format("Yaw:%.1f[105-165]  Pitch:%.1f[105-165]  Inv:%c%c", 
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

        if (ga_online_enabled) {
            const char* phase_text = (ga_phase == OnlineGaPhase::EvaluateGeneration) ? "EVAL" : "BEST_WAIT_ENTER";
            const int eval_idx = std::min(ga_candidate_index + 1, std::max(1, ga_population_size));
            cv::putText(canvas,
                        cv::format("GA:%s Gen:%d Candidate:%d/%d Test:%.0fs",
                                   phase_text, ga_generation, eval_idx, ga_population_size, ga_eval_seconds),
                        cv::Point(16, 132),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.5,
                        cv::Scalar(170, 255, 170),
                        1);
        }

        cv::putText(canvas,
                    use_stream ? "Input: stream" : "Input: camera",
                    cv::Point(16, canvas.rows - 60),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.45,
                    cv::Scalar(180, 220, 255),
                    1);

        cv::putText(canvas,
                    ga_online_enabled
                        ? "Q:quit  Enter:next generation  I/K:inv pitch  J/L:inv yaw"
                        : "Q:quit  T:toggle tracking  I/K:inv pitch  J/L:inv yaw  R:reset PID",
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

        int key = -1;
        if (gui_enabled) {
            try {
                cv::imshow(win_name, canvas);
                key = cv::waitKey(1);
            } catch (const cv::Exception& e) {
                gui_enabled = false;
                std::cerr << "OpenCV GUI runtime error, disabling display: " << e.what() << std::endl;
            }
        }
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }

        if (ga_online_enabled && (key == 10 || key == 13) && ga_phase == OnlineGaPhase::RunBestAndWaitEnter) {
            ga_generation++;
            ga_candidate_index = 0;
            build_generation(ga_seed);
            apply_genome(ga_population.front().genome);
            pid_yaw.output_limit = max_output_deg;
            pid_pitch.output_limit = max_output_deg;
            filter_initialized = false;
            reset_ga_metrics();
            reset_pose_to_center();
            ga_eval_start = std::chrono::steady_clock::now();
            ga_phase = OnlineGaPhase::EvaluateGeneration;
            std::cout << "[GA] Start generation " << ga_generation << std::endl;
        }

        if (ga_online_enabled) {
            continue;
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
    if (fixed_pid_log.is_open()) {
        fixed_pid_log.flush();
        fixed_pid_log.close();
    }
    cap.release();
    cv::destroyAllWindows();
    
    std::cout << "Program terminated. Total adjustments - Yaw: " << total_yaw_adjust 
              << "deg, Pitch: " << total_pitch_adjust << "deg" << std::endl;
    
    return 0;
}