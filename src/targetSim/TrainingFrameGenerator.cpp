#include "TrainingFrameGenerator.hpp"
#include <cmath>
#include <iostream>
#include <random>
#include <algorithm>
#include <chrono>

using namespace std;
using namespace cv;
TrainingFrameGenerator::TrainingFrameGenerator(int width, int height, float fps)
    : _target_sim(width, height), _fps(fps), _current_time(0.0f),
      _current_mode(MODE_PENTAGON_ROTATION), _paused(false),
      _param1(0.0f), _param2(0.0f),
      _angular_velocity_func(nullptr),
      _parametric_duration(0.0f), _loop_motion(true),
      _current_pentagon_center(-1, -1),  // 初始化为无效值
      _need_regenerate_pentagon(true),   // 需要重新生成
      _current_pentagon_angle(0.0f)      // 初始角度为0
{
    std::cout << "Training Frame Generator Initialized: " 
              << width << "x" << height 
              << " @ " << fps << " FPS" << std::endl;
    
    // 默认中心为图像中心
    _current_pentagon_center = _target_sim.get_center();
    
    // 注意：TargetSim 类没有 set_target_color 方法
    // 我们可以通过生成一个单色块帧来初始化颜色
    // 或者让 TargetSim 在第一次调用时自己初始化
    
    std::cout << "Initial pentagon center: (" 
              << _current_pentagon_center.x << ", " 
              << _current_pentagon_center.y << ")" << std::endl;
    
    set_training_mode(MODE_PENTAGON_ROTATION, 1.0f);
    reset();
}

void TrainingFrameGenerator::set_training_mode(TrainingMode mode, 
                                              float param1, 
                                              float param2,
                                              AngularVelocityFunction angular_velocity_func) {
    _current_mode = mode;
    _param1 = param1;
    _param2 = param2;
    _angular_velocity_func = angular_velocity_func;
    _paused = false;
    
    // 如果是五角星旋转模式，确保有有效的中心位置
    if (mode == MODE_PENTAGON_ROTATION) {
        // 如果当前中心位置无效，设置为图像中心
        if (_current_pentagon_center.x < 0 || _current_pentagon_center.y < 0) {
            _current_pentagon_center = _target_sim.get_center();
        }
        
        // 重置五角星角度
        _current_pentagon_angle = 0.0f;
        _need_regenerate_pentagon = true;  // 模式切换时需要重新生成
    }
    
    std::cout << "\nSet training mode: ";
    switch (mode) {
        case MODE_PENTAGON_ROTATION:// 对于五角星旋转模式，参数1用于固定角速度，参数二无效，如果设定了角速度参数方程，则使用参数方程
            std::cout << "Pentagon Rotation Mode" << std::endl;
            if (_angular_velocity_func) {
                std::cout << "  Using custom angular velocity function" << std::endl;
            } else {
                std::cout << "  Angular Velocity: " << param1 << " rad/s (constant)" << std::endl;
            }
            std::cout << "  Center: (" << _current_pentagon_center.x 
                      << ", " << _current_pentagon_center.y << ")" << std::endl;
            std::cout << "  Use get_next_frame(pentagon_center) to change position" << std::endl;
            std::cout << "  Use regenerate_pentagon(center) for new random pentagon" << std::endl;
            break;
        case MODE_LINEAR_MOVEMENT:// 对于线性运动模式，参数1和参数2分别表示x和y方向的速度（像素/秒）
            std::cout << "Linear Movement Mode" << std::endl;
            std::cout << "  Velocity: (" << param1 << ", " << param2 << ") px/s" << std::endl;
            break;
        case MODE_RANDOM_APPEARANCE:// 对于随机出现模式，参数1表示更换间隔时间（秒），参数2无效
            std::cout << "Random Appearance Mode" << std::endl;
            std::cout << "  Interval: " << param1 << " seconds" << std::endl;
            break;
        case MODE_PARAMETRIC_MOTION:// 对于参数方程运动模式，参数1和参数2无效，需调用 set_parametric_motion_mode 设置方程
            std::cout << "Parametric Motion Mode" << std::endl;
            std::cout << "  Using custom parametric functions" << std::endl;
            break;
        default:
            std::cout << "Unknown Mode" << std::endl;
            break;
    }
    
    reset();
}

void TrainingFrameGenerator::set_parametric_motion_mode(ParametricFunction x_func, 
                                                       ParametricFunction y_func,
                                                       float duration,
                                                       bool loop) {
    _current_mode = MODE_PARAMETRIC_MOTION;
    _x_function = x_func;
    _y_function = y_func;
    _parametric_duration = duration;
    _loop_motion = loop;
    _paused = false;
    
    reset();
}

void TrainingFrameGenerator::set_circular_motion(const cv::Point2f& center, 
                                                float radius, 
                                                float angular_speed,
                                                bool loop) {
    _x_function = [center, radius, angular_speed](float t) -> float {
        return center.x + radius * std::cos(angular_speed * t);
    };
    
    _y_function = [center, radius, angular_speed](float t) -> float {
        return center.y + radius * std::sin(angular_speed * t);
    };
    
    _current_mode = MODE_PARAMETRIC_MOTION;
    _parametric_duration = 2 * M_PI / angular_speed;
    _loop_motion = loop;
    _paused = false;
    
    reset();
}

void TrainingFrameGenerator::set_sine_motion(const cv::Point2f& start_point, 
                                            float amplitude, 
                                            float frequency,
                                            float direction,
                                            float speed,
                                            bool loop,
                                            float duration) {
    float omega = 2 * M_PI * frequency;
    float dir_rad = direction * M_PI / 180.0f;
    
    // 方向向量
    float dx = std::cos(dir_rad);
    float dy = std::sin(dir_rad);
    
    // 垂直方向向量
    float perp_dx = -dy;
    float perp_dy = dx;
    
    _x_function = [start_point, amplitude, omega, dx, perp_dx, speed](float t) -> float {
        float main_motion = dx * speed * t;
        float oscillation = perp_dx * amplitude * std::sin(omega * t);
        return start_point.x + main_motion + oscillation;
    };
    
    _y_function = [start_point, amplitude, omega, dy, perp_dy, speed](float t) -> float {
        float main_motion = dy * speed * t;
        float oscillation = perp_dy * amplitude * std::sin(omega * t);
        return start_point.y + main_motion + oscillation;
    };
    
    _current_mode = MODE_PARAMETRIC_MOTION;
    _parametric_duration = duration;
    _loop_motion = loop;
    _paused = false;
    
    reset();
}

void TrainingFrameGenerator::set_lissajous_motion(const cv::Point2f& center, 
                                                 float a, float b,
                                                 float wx, float wy, 
                                                 float phase,
                                                 bool loop,
                                                 float duration) {
    _x_function = [center, a, wx, phase](float t) -> float {
        return center.x + a * std::sin(wx * t + phase);
    };
    
    _y_function = [center, b, wy](float t) -> float {
        return center.y + b * std::sin(wy * t);
    };
    
    _current_mode = MODE_PARAMETRIC_MOTION;
    _parametric_duration = duration;
    _loop_motion = loop;
    _paused = false;
    
    reset();
}

TrainingFrameGenerator::TrainingFrame 
TrainingFrameGenerator::get_next_frame(float timestamp, 
                                      const cv::Point2f& pentagon_center) {
    // 如果指定了新的中心位置（仅在五角星旋转模式下有效）
    if (_current_mode == MODE_PENTAGON_ROTATION) {
        if (pentagon_center.x != -1 || pentagon_center.y != -1) {
            _current_pentagon_center = pentagon_center;
            _need_regenerate_pentagon = true;
        }
    }
    
    // 更新时间（仅在未暂停时）
    if (!_paused) {
        if (timestamp < 0) {
            // 使用内部时间
            _current_time += 1.0f / _fps;
        } else {
            // 使用指定的时间戳
            _current_time = timestamp;
        }
    } else {
        // 暂停时，如果指定了时间戳就使用它，否则保持当前时间
        if (timestamp >= 0) {
            _current_time = timestamp;
        }
        // 否则保持 _current_time 不变
    }
    
    // Generate frame based on current mode
    switch (_current_mode) {
        case MODE_PENTAGON_ROTATION:
            return _generate_pentagon_rotation(_current_time);
        case MODE_LINEAR_MOVEMENT:
            return _generate_linear_movement(_current_time);
        case MODE_RANDOM_APPEARANCE:
            return _generate_random_appearance(_current_time);
        case MODE_PARAMETRIC_MOTION:
            return _generate_parametric_motion(_current_time);
        default:
            return _generate_pentagon_rotation(_current_time);
    }
}

void TrainingFrameGenerator::regenerate_pentagon(const cv::Point2f& center) {
    if (center.x != -1 || center.y != -1) {
        // 使用指定的中心位置
        _current_pentagon_center = center;
    } else {
        // 使用随机位置
        _current_pentagon_center = _target_sim.get_random_position(150.0f);
    }
    
    // 强制重新生成颜色
    _target_sim.regenerate_colors();
    
    // 重置角度
    _current_pentagon_angle = 0.0f;
    _need_regenerate_pentagon = true;
    
    std::cout << "Pentagon regenerated at: (" 
              << _current_pentagon_center.x << ", " 
              << _current_pentagon_center.y << ")" << std::endl;
}

void TrainingFrameGenerator::reset() {
    _current_time = 0.0f;
    _last_appear_time = 0.0f;
    _current_position = _target_sim.get_center();
    _last_random_position = _target_sim.get_center();
    _paused = false;
    
    // 重置五角星角度
    _current_pentagon_angle = 0.0f;
    _need_regenerate_pentagon = true;
}

cv::Point2f TrainingFrameGenerator::_calculate_parametric_position(float t) {
    if (!_x_function || !_y_function) {
        return _target_sim.get_center();
    }
    
    // 如果循环运动且周期大于0，对时间取模
    if (_loop_motion && _parametric_duration > 0) {
        t = std::fmod(t, _parametric_duration);
    }
    
    float x = _x_function(t);
    float y = _y_function(t);
    
    return cv::Point2f(x, y);
}

cv::Point2f TrainingFrameGenerator::_calculate_parametric_velocity(float t, float dt) {
    if (!_x_function || !_y_function) {
        return cv::Point2f(0, 0);
    }
    
    cv::Point2f pos_t = _calculate_parametric_position(t);
    cv::Point2f pos_t_dt = _calculate_parametric_position(t + dt);
    
    return cv::Point2f(
        (pos_t_dt.x - pos_t.x) / dt,
        (pos_t_dt.y - pos_t.y) / dt
    );
}

TrainingFrameGenerator::TrainingFrame 
TrainingFrameGenerator::_generate_pentagon_rotation(float timestamp) {
    TrainingFrame frame_data;
    
    // 计算当前角速度
    float current_angular_velocity = 0.0f;
    if (_angular_velocity_func) {
        // 使用自定义角速度函数
        current_angular_velocity = _angular_velocity_func(timestamp);
    } else {
        // 使用恒定角速度
        current_angular_velocity = _param1;
    }
    
    // 计算旋转角度 - 基于时间戳，而不是累积
    // 角度 = 角速度 × 时间
    _current_pentagon_angle = current_angular_velocity * timestamp;
    
    // 规范化角度
    float display_angle = std::fmod(_current_pentagon_angle, 2 * M_PI);
    
    // 生成五角星图像
    cv::Point2f target_pos;
    frame_data.frame = _target_sim.generate_pentagon_frame(
        _current_pentagon_center, 
        display_angle, 
        &target_pos);
    
    // 获取目标信息
    frame_data.target_position = target_pos;
    frame_data.timestamp = timestamp;
    
    // 计算线速度（v = ω × r）
    cv::Point2f center_to_target = frame_data.target_position - _current_pentagon_center;
    float radius = cv::norm(center_to_target);
    
    if (radius > 0.001f) {
        // 切线方向
        cv::Point2f tangent(-center_to_target.y / radius, center_to_target.x / radius);
        float linear_speed = current_angular_velocity * radius;
        frame_data.velocity = tangent * linear_speed;
    } else {
        frame_data.velocity = cv::Point2f(0, 0);
    }
    
    // 重置重新生成标志
    _need_regenerate_pentagon = false;
    
    // 调试输出
    if (static_cast<int>(timestamp * _fps) % 60 == 0) {  // 每2秒输出一次
        std::cout << "[TrainingFrame] t=" << timestamp 
                  << "s, Center: (" << _current_pentagon_center.x 
                  << ", " << _current_pentagon_center.y << ")"
                  << ", ω=" << current_angular_velocity << " rad/s"
                  << ", θ=" << display_angle << " rad"
                  << ", Target: (" << frame_data.target_position.x 
                  << ", " << frame_data.target_position.y << ")"
                  << ", v=" << cv::norm(frame_data.velocity) << " px/s" << std::endl;
    }
    
    return frame_data;
}

TrainingFrameGenerator::TrainingFrame 
TrainingFrameGenerator::_generate_linear_movement(float timestamp) {
    TrainingFrame frame_data;
    
    // 边界参数
    float margin = 120.0f;
    int width = _target_sim.get_width();
    int height = _target_sim.get_height();
    
    // 使用静态变量跟踪速度和位置
    static cv::Point2f current_velocity(_param1, _param2);
    static cv::Point2f current_position = _target_sim.get_center();
    static float last_timestamp = 0.0f;
    
    // 如果时间重置，重新初始化
    if (timestamp <= 0.0f || timestamp < last_timestamp) {
        current_velocity = cv::Point2f(_param1, _param2);
        current_position = _target_sim.get_center();
    }
    
    // 计算时间增量
    float dt = (last_timestamp == 0.0f) ? 0.0f : (timestamp - last_timestamp);
    last_timestamp = timestamp;
    
    // 更新位置
    current_position.x += current_velocity.x * dt;
    current_position.y += current_velocity.y * dt;
    
    // 检查并处理边界碰撞
    if (current_position.x < margin) {
        current_position.x = margin;
        current_velocity.x = std::abs(current_velocity.x);
    } else if (current_position.x > width - margin) {
        current_position.x = width - margin;
        current_velocity.x = -std::abs(current_velocity.x);
    }
    
    if (current_position.y < margin) {
        current_position.y = margin;
        current_velocity.y = std::abs(current_velocity.y);
    } else if (current_position.y > height - margin) {
        current_position.y = height - margin;
        current_velocity.y = -std::abs(current_velocity.y);
    }
    
    // 确保位置在边界内
    _current_position.x = std::max(margin, std::min(current_position.x, static_cast<float>(width - margin)));
    _current_position.y = std::max(margin, std::min(current_position.y, static_cast<float>(height - margin)));
    
    // 生成单色块目标
    frame_data.frame = _target_sim.generate_single_blob_frame(
        _current_position, 40, false, cv::Scalar(-1, -1, -1));
    
    // 获取目标位置
    frame_data.target_position = _target_sim.get_last_target_position();
    frame_data.timestamp = timestamp;
    frame_data.velocity = current_velocity;
    
    return frame_data;
}

TrainingFrameGenerator::TrainingFrame 
TrainingFrameGenerator::_generate_random_appearance(float timestamp) {
    TrainingFrame frame_data;
    
    // 检查是否需要更新位置
    if (timestamp - _last_appear_time >= _param1 || _last_appear_time == 0.0f) {
        _last_random_position = _target_sim.get_random_position(150.0f);
        _last_appear_time = timestamp;
    }
    
    // 生成单色块目标
    frame_data.frame = _target_sim.generate_single_blob_frame(
        _last_random_position, 40, false, cv::Scalar(-1, -1, -1));
    
    // 获取目标位置
    frame_data.target_position = _target_sim.get_last_target_position();
    frame_data.timestamp = timestamp;
    frame_data.velocity = cv::Point2f(0, 0);
    
    return frame_data;
}

TrainingFrameGenerator::TrainingFrame 
TrainingFrameGenerator::_generate_parametric_motion(float timestamp) {
    TrainingFrame frame_data;
    
    // 计算当前位置
    _current_position = _calculate_parametric_position(timestamp);
    
    // 计算速度
    cv::Point2f velocity = _calculate_parametric_velocity(timestamp);
    
    // 生成单色块目标
    frame_data.frame = _target_sim.generate_single_blob_frame(
        _current_position, 40, false, cv::Scalar(-1, -1, -1));
    
    // 获取目标位置
    frame_data.target_position = _target_sim.get_last_target_position();
    frame_data.timestamp = timestamp;
    frame_data.velocity = velocity;
    
    return frame_data;
}