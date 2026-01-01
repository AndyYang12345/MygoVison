// targetStreamSimulator.cpp - 修正版本
#include "targetStreamSimulator.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>

// 构造函数
TargetStreamSimulator::TargetStreamSimulator(int width, int height, 
                                             float fps, cv::Scalar background)
    : _target_sim(width, height, background), 
      _fps(fps), 
      _current_time(0.0f),
      _canvas_width(width),
      _canvas_height(height),
      _canvas_background(background),
      _pattern("pentagon"),
      _center_color(0, 0, 255),
      _target_idx(0),
      _current_center(width / 2.0f, height / 2.0f),
      _current_rotation(0.0f),
      _is_single_blob(false),
      _blob_size(30),
      _use_random_colors(false),
      _color_change_rate(0.0f),
      _last_color_change_time(0.0f) {
    
    std::cout << "通用视频流模拟器初始化: " 
              << width << "x" << height 
              << " @ " << fps << " FPS" << std::endl;
    
    _position_func = [this](float t) -> cv::Point2f {
        return cv::Point2f(_canvas_width / 2.0f, _canvas_height / 2.0f);
    };
    
    _rotation_func = [](float t) -> float { return 0.0f; };
}

// 更新当前状态
void TargetStreamSimulator::_update_current_state(float timestamp) {
    // 更新当前位置
    if (_position_func) {
        _current_center = _position_func(timestamp);
    }
    
    // 更新当前旋转角度
    if (_rotation_func) {
        _current_rotation = _rotation_func(timestamp);
        // 归一化到 [0, 2π)
        while (_current_rotation >= 2 * M_PI) {
            _current_rotation -= 2 * M_PI;
        }
        while (_current_rotation < 0) {
            _current_rotation += 2 * M_PI;
        }
    }
}

// 获取指定时间戳的帧 - 修正版本
cv::Mat TargetStreamSimulator::get_frame_at_time(float timestamp) {
    // 更新时间戳
    _current_time = timestamp;
    
    // 更新当前状态
    _update_current_state(timestamp);
    
    // 更新随机颜色（如果需要）
    if (_use_random_colors && _color_change_rate > 0) {
        _update_random_colors(timestamp);
    }
    
    // 根据图案类型生成帧
    cv::Mat frame = _generate_frame_for_pattern(timestamp);
    
    return frame;
}

// 生成单色块帧 - 修正版本
cv::Mat TargetStreamSimulator::_generate_single_blob(float timestamp) {
    cv::Mat image(_canvas_height, _canvas_width, CV_8UC3, _canvas_background);
    
    cv::circle(image, _current_center, _blob_size, _center_color, -1);
    
    return image;
}

// 生成多色块帧 - 修正版本
cv::Mat TargetStreamSimulator::_generate_multi_blob(float timestamp) {
    // 获取颜色
    std::vector<cv::Scalar> colors;
    if (_color_func) {
        colors = _color_func(timestamp);
    } else if (_use_random_colors) {
        colors = _current_random_colors;
    } else {
        colors = {
            cv::Scalar(0, 255, 0),
            cv::Scalar(255, 0, 0),
            cv::Scalar(255, 255, 0),
            cv::Scalar(255, 0, 255),
            cv::Scalar(0, 255, 255)
        };
    }
    
    // 使用TargetSim生成帧
    return _target_sim.generate_frame(
        _center_color,
        colors,
        _pattern,
        _current_center,
        _current_rotation,
        _target_idx
    );
}

// 更新随机颜色
void TargetStreamSimulator::_update_random_colors(float timestamp) {
    if (timestamp - _last_color_change_time >= _color_change_rate) {
        _current_random_colors = _target_sim.get_random_colors(5);
        _last_color_change_time = timestamp;
    }
}

// 其他函数保持不变...
void TargetStreamSimulator::configure_basic(const cv::Scalar& center_color,
                                           const std::string& pattern,
                                           int target_idx) {
    _center_color = center_color;
    _pattern = pattern;
    _target_idx = target_idx;
    _is_single_blob = (pattern == "single_blob");
    
    std::cout << "基础配置: 图案=" << pattern 
              << ", 中心颜色=BGR(" << center_color[0] << "," 
              << center_color[1] << "," << center_color[2] << ")"
              << ", 目标索引=" << target_idx << std::endl;
}

void TargetStreamSimulator::configure_fixed_color_motion(
    const std::vector<cv::Scalar>& surround_colors,
    RotationFunction rotation_func,
    PositionFunction position_func) {
    
    _use_random_colors = false;
    
    _color_func = [surround_colors](float t) -> std::vector<cv::Scalar> {
        return surround_colors;
    };
    
    if (rotation_func) {
        _rotation_func = rotation_func;
    }
    
    if (position_func) {
        _position_func = position_func;
    }
    
    std::cout << "固定颜色运动配置完成" << std::endl;
}

void TargetStreamSimulator::configure_random_color_motion(
    RotationFunction rotation_func,
    PositionFunction position_func,
    float color_change_rate) {
    
    _use_random_colors = true;
    _color_change_rate = color_change_rate;
    
    _current_random_colors = _target_sim.get_random_colors(5);
    
    _color_func = [this](float t) -> std::vector<cv::Scalar> {
        return _current_random_colors;
    };
    
    if (rotation_func) {
        _rotation_func = rotation_func;
    }
    
    if (position_func) {
        _position_func = position_func;
    }
    
    std::cout << "随机颜色运动配置完成，颜色变化率: " 
              << color_change_rate << " 秒/次" << std::endl;
}

void TargetStreamSimulator::configure_custom_color_motion(
    ColorFunction color_func,
    RotationFunction rotation_func,
    PositionFunction position_func) {
    
    _use_random_colors = false;
    _color_func = color_func;
    
    if (rotation_func) {
        _rotation_func = rotation_func;
    }
    
    if (position_func) {
        _position_func = position_func;
    }
    
    std::cout << "自定义颜色运动配置完成" << std::endl;
}

void TargetStreamSimulator::configure_single_blob_motion(
    const cv::Scalar& color,
    PositionFunction position_func,
    int size) {
    
    _is_single_blob = true;
    _pattern = "single_blob";
    _center_color = color;
    _blob_size = size;
    
    if (position_func) {
        _position_func = position_func;
    }
    
    std::cout << "单色块运动配置完成，大小: " << size << " 像素" << std::endl;
}

// 预定义运动函数实现
TargetStreamSimulator::PositionFunction 
TargetStreamSimulator::linear_motion(cv::Point2f start, cv::Point2f velocity) {
    return [start, velocity](float t) -> cv::Point2f {
        return cv::Point2f(start.x + velocity.x * t,
                          start.y + velocity.y * t);
    };
}

TargetStreamSimulator::PositionFunction 
TargetStreamSimulator::circular_motion(cv::Point2f center, float radius, float angular_speed) {
    return [center, radius, angular_speed](float t) -> cv::Point2f {
        float angle = angular_speed * t;
        return cv::Point2f(center.x + radius * std::cos(angle),
                          center.y + radius * std::sin(angle));
    };
}

TargetStreamSimulator::PositionFunction 
TargetStreamSimulator::sinusoidal_motion(cv::Point2f start, float amplitude, float frequency) {
    return [start, amplitude, frequency](float t) -> cv::Point2f {
        return cv::Point2f(start.x + amplitude * std::sin(2 * M_PI * frequency * t),
                          start.y + amplitude * std::cos(2 * M_PI * frequency * t));
    };
}

TargetStreamSimulator::RotationFunction 
TargetStreamSimulator::constant_rotation(float speed) {
    return [speed](float t) -> float {
        return speed * t;
    };
}

TargetStreamSimulator::RotationFunction 
TargetStreamSimulator::oscillating_rotation(float amplitude, float frequency) {
    return [amplitude, frequency](float t) -> float {
        return amplitude * std::sin(2 * M_PI * frequency * t);
    };
}

cv::Mat TargetStreamSimulator::get_next_frame() {
    float next_time = _current_time + 1.0f / _fps;
    return get_frame_at_time(next_time);
}

void TargetStreamSimulator::reset() {
    _current_time = 0.0f;
    _last_color_change_time = 0.0f;
    _current_center = cv::Point2f(_canvas_width / 2.0f, _canvas_height / 2.0f);
    _current_rotation = 0.0f;
    
    if (_use_random_colors) {
        _current_random_colors = _target_sim.get_random_colors(5);
    }
    
    std::cout << "模拟器已重置" << std::endl;
}

cv::Mat TargetStreamSimulator::_generate_frame_for_pattern(float timestamp) {
    if (_is_single_blob) {
        return _generate_single_blob(timestamp);
    } else {
        return _generate_multi_blob(timestamp);
    }
}