#include "TrainingFrameGenerator.hpp"
#include <cmath>
#include <iostream>
#include <random>
#include <algorithm>
#include <numeric>

TrainingFrameGenerator::TrainingFrameGenerator(int width, int height, float fps)
    : _target_sim(width, height), _fps(fps), _current_time(0.0f),
      _current_mode(MODE_PENTAGON_ROTATION), _paused(false),
      _param1(0.0f), _param2(0.0f),
      _parametric_duration(0.0f), _loop_motion(true) {
    
    std::cout << "Training Frame Generator Initialized: " 
              << width << "x" << height 
              << " @ " << fps << " FPS" << std::endl;
    
    // 为单色块模式初始化一个随机颜色
    cv::Scalar initial_color = TargetSim::COLOR_TABLE[rand() % TargetSim::COLOR_TABLE.size()];
    _target_sim.set_target_color(initial_color);
    
    std::cout << "Initial target color: BGR(" 
              << initial_color[0] << ", "
              << initial_color[1] << ", "
              << initial_color[2] << ")" << std::endl;
    
    set_training_mode(MODE_PENTAGON_ROTATION, 1.0f);
    reset();
}

void TrainingFrameGenerator::set_training_mode(TrainingMode mode, float param1, float param2) {
    _current_mode = mode;
    _param1 = param1;
    _param2 = param2;
    _paused = false;
    
    std::cout << "\nSet training mode: ";
    switch (mode) {
        case MODE_PENTAGON_ROTATION:
            std::cout << "Pentagon Rotation Mode" << std::endl;
            std::cout << "  Angular Velocity: " << param1 << " rad/s" << std::endl;
            break;
        case MODE_LINEAR_MOVEMENT:
            std::cout << "Linear Movement Mode" << std::endl;
            std::cout << "  Velocity: (" << param1 << ", " << param2 << ") px/s" << std::endl;
            break;
        case MODE_RANDOM_APPEARANCE:
            std::cout << "Random Appearance Mode" << std::endl;
            std::cout << "  Interval: " << param1 << " seconds" << std::endl;
            break;
        case MODE_PARAMETRIC_MOTION:
            std::cout << "Parametric Motion Mode" << std::endl;
            std::cout << "  Using custom parametric functions" << std::endl;
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
    
    std::cout << "\nSet parametric motion mode:" << std::endl;
    std::cout << "  Duration: " << duration << " seconds" << std::endl;
    std::cout << "  Loop: " << (loop ? "enabled" : "disabled") << std::endl;
    
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
    _parametric_duration = 2 * M_PI / angular_speed;  // 一个完整周期的时间
    _loop_motion = loop;
    _paused = false;
    
    std::cout << "\nSet circular motion:" << std::endl;
    std::cout << "  Center: (" << center.x << ", " << center.y << ")" << std::endl;
    std::cout << "  Radius: " << radius << std::endl;
    std::cout << "  Angular speed: " << angular_speed << " rad/s" << std::endl;
    std::cout << "  Period: " << _parametric_duration << " seconds" << std::endl;
    std::cout << "  Loop: " << (loop ? "enabled" : "disabled") << std::endl;
    
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
    
    // 垂直方向向量（用于振幅方向）
    float perp_dx = -dy;
    float perp_dy = dx;
    
    // 改进的正弦运动：在垂直于运动方向的方向上做正弦振荡
    _x_function = [start_point, amplitude, omega, dx, perp_dx, speed](float t) -> float {
        // 主运动方向 + 垂直方向的振荡
        float main_motion = dx * speed * t;  // 沿运动方向匀速运动
        float oscillation = perp_dx * amplitude * std::sin(omega * t);
        return start_point.x + main_motion + oscillation;
    };
    
    _y_function = [start_point, amplitude, omega, dy, perp_dy, speed](float t) -> float {
        // 主运动方向 + 垂直方向的振荡
        float main_motion = dy * speed * t;  // 沿运动方向匀速运动
        float oscillation = perp_dy * amplitude * std::sin(omega * t);
        return start_point.y + main_motion + oscillation;
    };
    
    _current_mode = MODE_PARAMETRIC_MOTION;
    _parametric_duration = duration;  // 设置较长的持续时间
    _loop_motion = loop;
    _paused = false;
    
    std::cout << "\nSet sine motion:" << std::endl;
    std::cout << "  Start point: (" << start_point.x << ", " << start_point.y << ")" << std::endl;
    std::cout << "  Amplitude: " << amplitude << std::endl;
    std::cout << "  Frequency: " << frequency << " Hz" << std::endl;
    std::cout << "  Direction: " << direction << " degrees" << std::endl;
    std::cout << "  Linear speed: " << speed << " px/s along direction" << std::endl;
    std::cout << "  Period: " << _parametric_duration << " seconds" << std::endl;
    std::cout << "  Loop: " << (loop ? "enabled" : "disabled") << std::endl;
    
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
    
    // 计算两个周期的最小公倍数作为总周期
    float period_x = 2 * M_PI / wx;
    float period_y = 2 * M_PI / wy;
    
    // 计算最小公倍数
    auto gcd = [](float a, float b) -> float {
        while (b != 0) {
            float temp = b;
            b = std::fmod(a, b);
            a = temp;
        }
        return a;
    };
    
    if (period_x > 0 && period_y > 0) {
        float g = gcd(period_x, period_y);
        _parametric_duration = period_x * period_y / g;
    } else {
        _parametric_duration = duration;  // 默认值
    }
    
    _loop_motion = loop;
    _paused = false;
    
    std::cout << "\nSet Lissajous motion:" << std::endl;
    std::cout << "  Center: (" << center.x << ", " << center.y << ")" << std::endl;
    std::cout << "  Amplitudes: (" << a << ", " << b << ")" << std::endl;
    std::cout << "  Frequencies: (" << wx << ", " << wy << ") rad/s" << std::endl;
    std::cout << "  Phase: " << phase << " rad" << std::endl;
    std::cout << "  Period: " << _parametric_duration << " seconds" << std::endl;
    std::cout << "  Loop: " << (loop ? "enabled" : "disabled") << std::endl;
    
    reset();
}

TrainingFrameGenerator::TrainingFrame TrainingFrameGenerator::get_next_frame(float timestamp) {
    // Update current time only if not paused
    if (!_paused) {
        if (timestamp < 0) {
            _current_time += 1.0f / _fps;
        } else {
            _current_time = timestamp;
        }
    } else {
        // If paused, use the provided timestamp or keep current time
        if (timestamp >= 0) {
            _current_time = timestamp;
        }
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

void TrainingFrameGenerator::reset() {
    _current_time = 0.0f;
    _last_appear_time = 0.0f;
    _current_position = _target_sim.get_center();
    _last_random_position = _target_sim.get_center();
    _paused = false;
}

cv::Point2f TrainingFrameGenerator::_calculate_parametric_position(float t) {
    if (!_x_function || !_y_function) {
        // 如果没有设置函数，返回中心位置
        return _target_sim.get_center();
    }
    
    // 如果循环运动且周期大于0，对时间取模
    if (_loop_motion && _parametric_duration > 0) {
        t = std::fmod(t, _parametric_duration);
    }
    
    // 存储上一次的位置（用于反弹计算）
    static cv::Point2f last_position = _target_sim.get_center();
    static bool first_call = true;
    
    float x = _x_function(t);
    float y = _y_function(t);
    
    cv::Point2f original_position(x, y);
    
    // 边界检查与反弹逻辑
    float margin = 80.0f;
    int width = _target_sim.get_width();
    int height = _target_sim.get_height();
    
    // 反弹处理标志
    bool bounced_x = false;
    bool bounced_y = false;
    
    // X方向边界处理
    if (x < margin) {
        // 从左侧反弹
        float overshoot = margin - x;
        x = margin + overshoot;  // 反弹到边界内侧
        bounced_x = true;
        std::cout << "[Boundary] Left bounce at x=" << x << std::endl;
    } else if (x > width - margin) {
        // 从右侧反弹
        float overshoot = x - (width - margin);
        x = (width - margin) - overshoot;  // 反弹到边界内侧
        bounced_x = true;
        std::cout << "[Boundary] Right bounce at x=" << x << std::endl;
    }
    
    // Y方向边界处理
    if (y < margin) {
        // 从顶部反弹
        float overshoot = margin - y;
        y = margin + overshoot;  // 反弹到边界内侧
        bounced_y = true;
        std::cout << "[Boundary] Top bounce at y=" << y << std::endl;
    } else if (y > height - margin) {
        // 从底部反弹
        float overshoot = y - (height - margin);
        y = (height - margin) - overshoot;  // 反弹到边界内侧
        bounced_y = true;
        std::cout << "[Boundary] Bottom bounce at y=" << y << std::endl;
    }
    
    // 如果不是第一次调用，记录反弹事件
    if (!first_call && (bounced_x || bounced_y)) {
        cv::Point2f bounce_position(x, y);
        
        // 计算反弹方向
        cv::Point2f incoming_vector = original_position - last_position;
        cv::Point2f outgoing_vector;
        
        if (bounced_x) {
            // X方向反弹，反转X分量
            outgoing_vector = cv::Point2f(-incoming_vector.x, incoming_vector.y);
        }
        if (bounced_y) {
            // Y方向反弹，反转Y分量
            outgoing_vector = cv::Point2f(incoming_vector.x, -incoming_vector.y);
        }
        if (bounced_x && bounced_y) {
            // 角落反弹，反转两个分量
            outgoing_vector = cv::Point2f(-incoming_vector.x, -incoming_vector.y);
            std::cout << "[Boundary] Corner bounce detected!" << std::endl;
        }
        
        // 添加能量损失（阻尼系数）
        float damping_factor = 1.0f;  // 保留100%的能量
        outgoing_vector *= damping_factor;
        
        std::cout << "[Boundary] Bounce event:" << std::endl;
        std::cout << "  Incoming: (" << incoming_vector.x << ", " << incoming_vector.y << ")" << std::endl;
        std::cout << "  Outgoing: (" << outgoing_vector.x << ", " << outgoing_vector.y << ")" << std::endl;
        std::cout << "  Damping factor: " << damping_factor << std::endl;
    }
    
    // 更新上一次位置
    last_position = cv::Point2f(x, y);
    first_call = false;
    
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
    
    // Calculate rotation angle
    float rotation_angle = _param1 * timestamp;  // param1 = angular velocity
    
    // Get a valid target index (0-4)
    int target_idx = rand() % 5;
    
    // Generate pentagon target with specific target index
    frame_data.frame = _target_sim.generate_pentagon_frame(
        _target_sim.get_center(), rotation_angle, target_idx);
    
    // Get the CORRECT target position from TargetSim
    frame_data.target_position = _target_sim.get_target_position();
    
    // Get the actual target index from TargetSim
    frame_data.target_index = target_idx;
    
    frame_data.timestamp = timestamp;
    frame_data.velocity = cv::Point2f(0, 0);  // 旋转模式速度设为0
    
    // Debug output (less frequent)
    if (static_cast<int>(timestamp * _fps) % 30 == 0) {  // 每秒输出一次
        std::cout << "[Pentagon Mode] Time: " << timestamp 
                  << "s, Angle: " << rotation_angle << " rad"
                  << ", Target Index: " << target_idx
                  << ", Position: (" << frame_data.target_position.x 
                  << ", " << frame_data.target_position.y << ")" << std::endl;
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
    bool bounced = false;
    
    // X方向碰撞
    if (current_position.x < margin) {
        current_position.x = margin;
        current_velocity.x = std::abs(current_velocity.x);  // 向右反弹
        bounced = true;
        std::cout << "[Linear-Bounce] Left wall at t=" << timestamp << std::endl;
    } else if (current_position.x > width - margin) {
        current_position.x = width - margin;
        current_velocity.x = -std::abs(current_velocity.x);  // 向左反弹
        bounced = true;
        std::cout << "[Linear-Bounce] Right wall at t=" << timestamp << std::endl;
    }
    
    // Y方向碰撞
    if (current_position.y < margin) {
        current_position.y = margin;
        current_velocity.y = std::abs(current_velocity.y);  // 向下反弹
        bounced = true;
        std::cout << "[Linear-Bounce] Top wall at t=" << timestamp << std::endl;
    } else if (current_position.y > height - margin) {
        current_position.y = height - margin;
        current_velocity.y = -std::abs(current_velocity.y);  // 向上反弹
        bounced = true;
        std::cout << "[Linear-Bounce] Bottom wall at t=" << timestamp << std::endl;
    }
    
    // 应用能量损失（如果发生了反弹）
    if (bounced) {
        float damping = 0.98f;  // 轻微的能量损失
        current_velocity *= damping;
        std::cout << "[Linear-Bounce] New velocity: (" 
                  << current_velocity.x << ", " << current_velocity.y << ") px/s" << std::endl;
    }
    
    // 确保位置在边界内（最终检查）
    _current_position.x = std::clamp(current_position.x, margin, static_cast<float>(width - margin));
    _current_position.y = std::clamp(current_position.y, margin, static_cast<float>(height - margin));
    
    // 生成单色块目标
    frame_data.frame = _target_sim.generate_single_blob_frame(
        _current_position, 40, false, cv::Scalar(-1, -1, -1));
    
    // 获取目标位置
    auto centroids = _target_sim.get_blob_centroids();
    frame_data.target_position = centroids.empty() ? _current_position : centroids[0];
    
    frame_data.target_index = 0;
    frame_data.timestamp = timestamp;
    frame_data.velocity = current_velocity;
    
    // 调试输出
    if (static_cast<int>(timestamp * _fps) % 30 == 0) {
        std::cout << "[Linear Mode] Time: " << timestamp 
                  << "s, Position: (" << _current_position.x 
                  << ", " << _current_position.y << ")"
                  << ", Velocity: (" << current_velocity.x
                  << ", " << current_velocity.y << ") px/s" << std::endl;
    }
    
    return frame_data;
}

TrainingFrameGenerator::TrainingFrame 
TrainingFrameGenerator::_generate_random_appearance(float timestamp) {
    TrainingFrame frame_data;
    
    // Check if we need to update position (based on interval)
    if (timestamp - _last_appear_time >= _param1 || _last_appear_time == 0.0f) {
        _last_random_position = _target_sim.get_random_position(150.0f);
        _last_appear_time = timestamp;
        
        std::cout << "[Random Mode] New position: (" 
                  << _last_random_position.x << ", " 
                  << _last_random_position.y << ")" << std::endl;
    }
    
    // Generate single blob target, use previous color (not random)
    frame_data.frame = _target_sim.generate_single_blob_frame(
        _last_random_position, 40, false, cv::Scalar(-1, -1, -1));
    
    // Get target position from TargetSim
    auto centroids = _target_sim.get_blob_centroids();
    if (!centroids.empty()) {
        frame_data.target_position = centroids[0];
    } else {
        frame_data.target_position = _last_random_position;
    }
    
    frame_data.target_index = 0;
    frame_data.timestamp = timestamp;
    frame_data.velocity = cv::Point2f(0, 0);  // 随机出现模式速度为0
    
    // Debug output (less frequent)
    if (static_cast<int>(timestamp * _fps) % 30 == 0) {
        std::cout << "[Random Mode] Time: " << timestamp 
                  << "s, Position: (" << _last_random_position.x 
                  << ", " << _last_random_position.y << ")" 
                  << ", Time since last: " << (timestamp - _last_appear_time) << "s" << std::endl;
    }
    
    return frame_data;
}

TrainingFrameGenerator::TrainingFrame 
TrainingFrameGenerator::_generate_parametric_motion(float timestamp) {
    TrainingFrame frame_data;
    
    // 计算当前位置
    _current_position = _calculate_parametric_position(timestamp);
    
    // 计算速度
    cv::Point2f velocity = _calculate_parametric_velocity(timestamp);
    
    // Generate single blob target
    frame_data.frame = _target_sim.generate_single_blob_frame(
        _current_position, 40, false, cv::Scalar(-1, -1, -1));
    
    // Get target position from TargetSim
    auto centroids = _target_sim.get_blob_centroids();
    if (!centroids.empty()) {
        frame_data.target_position = centroids[0];
    } else {
        frame_data.target_position = _current_position;
    }
    
    frame_data.target_index = 0;
    frame_data.timestamp = timestamp;
    frame_data.velocity = velocity;
    
    // Debug output (less frequent)
    if (static_cast<int>(timestamp * _fps) % 30 == 0) {
        std::cout << "[Parametric Motion] Time: " << timestamp << "s"
                  << ", Position: (" << _current_position.x << ", " << _current_position.y << ")"
                  << ", Velocity: (" << velocity.x << ", " << velocity.y << ") px/s" << std::endl;
    }
    
    return frame_data;
}