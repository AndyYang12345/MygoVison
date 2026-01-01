// TrainingFrameGenerator.cpp
#include "TrainingFrameGenerator.hpp"
#include <cmath>
#include <iostream>

TrainingFrameGenerator::TrainingFrameGenerator(int width, int height, float fps)
    : _target_sim(width, height), _fps(fps), _current_time(0.0f),
      _current_mode(MODE_PENTAGON_ROTATION), _param1(0.0f), _param2(0.0f) {
    
    std::cout << "训练帧生成器初始化: " 
              << width << "x" << height 
              << " @ " << fps << " FPS" << std::endl;
    
    // 默认模式：五角星旋转，角速度1 rad/s
    set_training_mode(MODE_PENTAGON_ROTATION, 1.0f);
    reset();
}

void TrainingFrameGenerator::set_training_mode(TrainingMode mode, float param1, float param2) {
    _current_mode = mode;
    _param1 = param1;
    _param2 = param2;
    
    std::cout << "设置训练模式: ";
    switch (mode) {
        case MODE_PENTAGON_ROTATION:
            std::cout << "五角星旋转模式, 角速度: " << param1 << " rad/s" << std::endl;
            break;
        case MODE_LINEAR_MOVEMENT:
            std::cout << "线性移动模式, 速度: (" << param1 << ", " << param2 << ") px/s" << std::endl;
            break;
        case MODE_RANDOM_APPEARANCE:
            std::cout << "随机出现模式, 出现间隔: " << param1 << " 秒" << std::endl;
            break;
    }
    
    reset();
}

TrainingFrameGenerator::TrainingFrame TrainingFrameGenerator::get_next_frame(float timestamp) {
    // 更新当前时间
    if (timestamp < 0) {
        _current_time += 1.0f / _fps;
    } else {
        _current_time = timestamp;
    }
    
    // 根据模式生成帧
    switch (_current_mode) {
        case MODE_PENTAGON_ROTATION:
            return _generate_pentagon_rotation(_current_time);
        case MODE_LINEAR_MOVEMENT:
            return _generate_linear_movement(_current_time);
        case MODE_RANDOM_APPEARANCE:
            return _generate_random_appearance(_current_time);
        default:
            return _generate_pentagon_rotation(_current_time);
    }
}

void TrainingFrameGenerator::reset() {
    _current_time = 0.0f;
    _last_appear_time = 0.0f;
    _current_position = _target_sim.get_center();
    _last_random_position = _target_sim.get_center();
}

// 五角星旋转模式
TrainingFrameGenerator::TrainingFrame 
TrainingFrameGenerator::_generate_pentagon_rotation(float timestamp) {
    TrainingFrame frame_data;
    
    // 计算旋转角度
    float rotation_angle = _param1 * timestamp;  // param1 = 角速度
    
    // 生成五角星靶子（固定位置在中心）
    frame_data.frame = _target_sim.generate_pentagon_frame(
        _target_sim.get_center(), rotation_angle);
    
    // 获取色块中心坐标
    auto centroids = _target_sim.get_blob_centroids();
    
    // 假设目标索引为0（实际上应该从_target_sim获取）
    frame_data.target_index = 0;
    
    // 目标位置是外围色块中与中心相同的那个
    // 注意：这里简化处理，实际应该根据颜色匹配确定
    if (centroids.size() > 1) {
        frame_data.target_position = centroids[1];  // 先使用第一个外围色块
    } else {
        frame_data.target_position = _target_sim.get_center();
    }
    
    frame_data.timestamp = timestamp;
    return frame_data;
}

// 线性移动模式
TrainingFrameGenerator::TrainingFrame 
TrainingFrameGenerator::_generate_linear_movement(float timestamp) {
    TrainingFrame frame_data;
    
    // 更新位置
    _current_position.x = _target_sim.get_center().x + _param1 * timestamp;
    _current_position.y = _target_sim.get_center().y + _param2 * timestamp;
    
    // 边界检查
    float margin = 120.0f;
    if (_current_position.x < margin) _current_position.x = margin;
    if (_current_position.x > _target_sim.get_width() - margin) _current_position.x = _target_sim.get_width() - margin;
    if (_current_position.y < margin) _current_position.y = margin;
    if (_current_position.y > _target_sim.get_height() - margin) _current_position.y = _target_sim.get_height() - margin;
    
    // 生成单色块目标
    frame_data.frame = _target_sim.generate_single_blob_frame(_current_position, 40);
    
    // 获取目标位置
    auto centroids = _target_sim.get_blob_centroids();
    if (!centroids.empty()) {
        frame_data.target_position = centroids[0];
    } else {
        frame_data.target_position = _current_position;
    }
    
    frame_data.target_index = 0;
    frame_data.timestamp = timestamp;
    return frame_data;
}

// 随机出现模式
TrainingFrameGenerator::TrainingFrame 
TrainingFrameGenerator::_generate_random_appearance(float timestamp) {
    TrainingFrame frame_data;
    
    // 检查是否需要更新位置
    if (timestamp - _last_appear_time >= _param1) {  // param1 = 出现间隔
        _last_random_position = _target_sim.get_random_position();
        _last_appear_time = timestamp;
    }
    
    // 生成单色块目标
    frame_data.frame = _target_sim.generate_single_blob_frame(_last_random_position, 40);
    
    // 获取目标位置
    auto centroids = _target_sim.get_blob_centroids();
    if (!centroids.empty()) {
        frame_data.target_position = centroids[0];
    } else {
        frame_data.target_position = _last_random_position;
    }
    
    frame_data.target_index = 0;
    frame_data.timestamp = timestamp;
    return frame_data;
}