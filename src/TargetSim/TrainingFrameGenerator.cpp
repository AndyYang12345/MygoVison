#include "TargetSim/TrainingFrameGenerator.hpp"
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
      _current_pentagon_center(-1, -1),
      _need_regenerate_pentagon(true),
      _current_pentagon_angle(0.0f),
      _performance_monitoring(false)
{
    std::cout << "Training Frame Generator Initialized: " 
              << width << "x" << height 
              << " @ " << fps << " FPS" << std::endl;
    
    // 初始化性能监控数据
    _perf_data.frame_count = 0;
    _perf_data.current_fps = 0.0f;
    _perf_data.avg_fps = 0.0f;
    _perf_data.min_fps = 9999.0f;
    _perf_data.max_fps = 0.0f;
    _perf_data.samples = 0;
    _perf_data.last_fps_time = std::chrono::high_resolution_clock::now();
    
    // 默认中心为图像中心
    _current_pentagon_center = _target_sim.get_center();
    
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
    
    //模式设置调试输出
    // std::cout << "\nSet training mode: "; 
    // switch (mode) {
    //     case MODE_PENTAGON_ROTATION:
    //         std::cout << "Pentagon Rotation Mode" << std::endl;
    //         if (_angular_velocity_func) {
    //             std::cout << "  Using custom angular velocity function" << std::endl;
    //         } else {
    //             std::cout << "  Angular Velocity: " << param1 << " rad/s (constant)" << std::endl;
    //         }
    //         std::cout << "  Center: (" << _current_pentagon_center.x 
    //                   << ", " << _current_pentagon_center.y << ")" << std::endl;
    //         break;
    //     case MODE_LINEAR_MOVEMENT:
    //         std::cout << "Linear Movement Mode" << std::endl;
    //         std::cout << "  Velocity: (" << param1 << ", " << param2 << ") px/s" << std::endl;
    //         break;
    //     case MODE_RANDOM_APPEARANCE:
    //         std::cout << "Random Appearance Mode" << std::endl;
    //         std::cout << "  Interval: " << param1 << " seconds" << std::endl;
    //         break;
    //     case MODE_PARAMETRIC_MOTION:
    //         std::cout << "Parametric Motion Mode" << std::endl;
    //         std::cout << "  Using custom parametric functions" << std::endl;
    //         break;
    //     default:
    //         std::cout << "Unknown Mode" << std::endl;
    //         break;
    // }
    
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
    
    float dx = std::cos(dir_rad);
    float dy = std::sin(dir_rad);
    
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
    // 开始帧计时
    _start_frame_timing();
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
    }
    
    
    // Generate frame based on current mode
    TrainingFrame frame_data;
    switch (_current_mode) {
        case MODE_PENTAGON_ROTATION:
            frame_data = _generate_pentagon_rotation(_current_time);
            break;
        case MODE_LINEAR_MOVEMENT:
            frame_data = _generate_linear_movement(_current_time);
            break;
        case MODE_RANDOM_APPEARANCE:
            frame_data = _generate_random_appearance(_current_time);
            break;
        case MODE_PARAMETRIC_MOTION:
            frame_data = _generate_parametric_motion(_current_time);
            break;
        default:
            frame_data = _generate_pentagon_rotation(_current_time);
    }
    return frame_data;
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
    
    // std::cout << "Pentagon regenerated at: (" 
    //           << _current_pentagon_center.x << ", " 
    //           << _current_pentagon_center.y << ")" << std::endl;
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
    
    // 重置性能数据
    _perf_data.frame_count = 0;
    _perf_data.current_fps = 0.0f;
    _perf_data.avg_fps = 0.0f;
    _perf_data.min_fps = 9999.0f;
    _perf_data.max_fps = 0.0f;
    _perf_data.samples = 0;
    _perf_data.last_fps_time = std::chrono::high_resolution_clock::now();
}

cv::Point2f TrainingFrameGenerator::_calculate_parametric_position(float t) {
    if (!_x_function || !_y_function) {
        return _target_sim.get_center();
    }
    
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
        current_angular_velocity = _angular_velocity_func(timestamp);
    } else {
        current_angular_velocity = _param1;
    }
    
    // === 修正：使用积分而不是直接乘法 ===
    // 静态变量记录上一次的时间戳和累积角度
    static float last_timestamp = 0.0f;
    static float accumulated_angle = 0.0f;
    
    // 如果是第一次调用或重置后
    if (timestamp <= 0.0f || timestamp < last_timestamp) {
        last_timestamp = timestamp;
        accumulated_angle = 0.0f;
    }
    
    // 计算时间增量并积分
    float dt = timestamp - last_timestamp;
    if (dt > 0) {
        accumulated_angle += current_angular_velocity * dt;
        last_timestamp = timestamp;
    }
    
    // 使用累积角度
    _current_pentagon_angle = accumulated_angle;
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
    
    // 计算线速度
    cv::Point2f center_to_target = frame_data.target_position - _current_pentagon_center;
    float radius = cv::norm(center_to_target);
    
    if (radius > 0.001f) {
        cv::Point2f tangent(-center_to_target.y / radius, center_to_target.x / radius);
        float linear_speed = current_angular_velocity * radius;
        frame_data.velocity = tangent * linear_speed;
    } else {
        frame_data.velocity = cv::Point2f(0, 0);
    }
    
    _need_regenerate_pentagon = false;
    
    return frame_data;
}

TrainingFrameGenerator::TrainingFrame 
TrainingFrameGenerator::_generate_linear_movement(float timestamp) {
    TrainingFrame frame_data;
    
    float margin = 120.0f;
    int width = _target_sim.get_width();
    int height = _target_sim.get_height();
    
    static cv::Point2f current_velocity(_param1, _param2);
    static cv::Point2f current_position = _target_sim.get_center();
    static float last_timestamp = 0.0f;
    
    if (timestamp <= 0.0f || timestamp < last_timestamp) {
        current_velocity = cv::Point2f(_param1, _param2);
        current_position = _target_sim.get_center();
    }
    
    float dt = (last_timestamp == 0.0f) ? 0.0f : (timestamp - last_timestamp);
    last_timestamp = timestamp;
    
    current_position.x += current_velocity.x * dt;
    current_position.y += current_velocity.y * dt;
    
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
    
    _current_position.x = std::max(margin, std::min(current_position.x, static_cast<float>(width - margin)));
    _current_position.y = std::max(margin, std::min(current_position.y, static_cast<float>(height - margin)));
    
    frame_data.frame = _target_sim.generate_single_blob_frame(
        _current_position, 40, false, cv::Scalar(-1, -1, -1));
    
    frame_data.target_position = _target_sim.get_last_target_position();
    frame_data.timestamp = timestamp;
    frame_data.velocity = current_velocity;
    
    return frame_data;
}

TrainingFrameGenerator::TrainingFrame 
TrainingFrameGenerator::_generate_random_appearance(float timestamp) {
    TrainingFrame frame_data;
    
    if (timestamp - _last_appear_time >= _param1 || _last_appear_time == 0.0f) {
        _last_random_position = _target_sim.get_random_position(150.0f);
        _last_appear_time = timestamp;
    }
    
    frame_data.frame = _target_sim.generate_single_blob_frame(
        _last_random_position, 40, false, cv::Scalar(-1, -1, -1));
    
    frame_data.target_position = _target_sim.get_last_target_position();
    frame_data.timestamp = timestamp;
    frame_data.velocity = cv::Point2f(0, 0);
    
    return frame_data;
}

TrainingFrameGenerator::TrainingFrame 
TrainingFrameGenerator::_generate_parametric_motion(float timestamp) {
    TrainingFrame frame_data;
    
    _current_position = _calculate_parametric_position(timestamp);
    cv::Point2f velocity = _calculate_parametric_velocity(timestamp);
    
    frame_data.frame = _target_sim.generate_single_blob_frame(
        _current_position, 40, false, cv::Scalar(-1, -1, -1));
    
    frame_data.target_position = _target_sim.get_last_target_position();
    frame_data.timestamp = timestamp;
    frame_data.velocity = velocity;
    
    return frame_data;
}

// 性能监控辅助函数实现
void TrainingFrameGenerator::_start_frame_timing() {
    _perf_data.frame_start = std::chrono::high_resolution_clock::now();
}

void TrainingFrameGenerator::_mark_generation_done() {
    _perf_data.generation_done = std::chrono::high_resolution_clock::now();
}

void TrainingFrameGenerator::_mark_rendering_done() {
    _perf_data.rendering_done = std::chrono::high_resolution_clock::now();
}

TrainingFrameGenerator::FrameMetrics TrainingFrameGenerator::_end_frame_timing() {
    auto frame_end = std::chrono::high_resolution_clock::now();
    
    FrameMetrics metrics;
    metrics.generation_time_ms = std::chrono::duration<float, std::milli>(
        _perf_data.generation_done - _perf_data.frame_start).count();
    metrics.rendering_time_ms = std::chrono::duration<float, std::milli>(
        _perf_data.rendering_done - _perf_data.generation_done).count();
    metrics.wait_time_ms = std::chrono::duration<float, std::milli>(
        frame_end - _perf_data.rendering_done).count();
    metrics.frame_time_ms = metrics.generation_time_ms + 
                           metrics.rendering_time_ms + 
                           metrics.wait_time_ms;
    
    // 计算FPS
    _perf_data.frame_count++;
    float elapsed = std::chrono::duration<float>(
        frame_end - _perf_data.last_fps_time).count();
    
    if (elapsed >= 1.0f) {
        metrics.fps = _perf_data.frame_count / elapsed;
        _perf_data.frame_count = 0;
        _perf_data.last_fps_time = frame_end;
        
        // 更新统计
        _perf_data.samples++;
        _perf_data.avg_fps = (_perf_data.avg_fps * (_perf_data.samples - 1) + metrics.fps) / _perf_data.samples;
        _perf_data.min_fps = std::min(_perf_data.min_fps, metrics.fps);
        _perf_data.max_fps = std::max(_perf_data.max_fps, metrics.fps);
        _perf_data.current_fps = metrics.fps;
        
        // 性能警告
        if (_fps > 0 && metrics.fps < _fps * 0.8f) {
            std::cout << "[PERF WARNING] Low FPS: " << metrics.fps 
                      << " (target: " << _fps << ")" << std::endl;
        }
    } else {
        metrics.fps = _perf_data.current_fps;
    }
    
    return metrics;
}