// targetSim.cpp
#include "targetSim.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <random>

using namespace cv;
using namespace std;

// 初始化静态颜色表
const vector<Scalar> TargetSim::COLOR_TABLE = {
    Scalar(116, 5, 202),    // 索引0
    Scalar(167, 1, 98),     // 索引1
    Scalar(7, 237, 19),     // 索引2
    Scalar(23, 51, 215),    // 索引3
    Scalar(241, 132, 251),  // 索引4
    Scalar(17, 168, 214),   // 索引5
    Scalar(135, 199, 246),  // 索引6
    Scalar(221, 66, 76),    // 索引7
    Scalar(216, 199, 167),  // 索引8
    Scalar(85, 152, 55),    // 索引9
    Scalar(57, 244, 231),   // 索引10
    Scalar(23, 3, 23)       // 索引11
};

TargetSim::TargetSim(int width, int height, Scalar background) 
    : _width(width), _height(height), _background(background) {
    srand(static_cast<unsigned int>(time(nullptr)));
    
    // 初始化当前颜色组合
    _generate_valid_color_combo();
}

// 生成符合要求的颜色组合
void TargetSim::_generate_valid_color_combo() {
    // 随机选择中心颜色（从12色中选）
    int center_idx = rand() % COLOR_TABLE.size();
    _current_center_color = COLOR_TABLE[center_idx];
    
    // 随机选择目标索引（0-4）
    _current_target_idx = rand() % 5;
    
    // 准备可用的颜色索引（排除中心颜色）
    vector<int> available_indices;
    for (size_t i = 0; i < COLOR_TABLE.size(); i++) {
        if (i != static_cast<size_t>(center_idx)) {
            available_indices.push_back(i);
        }
    }
    
    // 打乱可用颜色
    random_shuffle(available_indices.begin(), available_indices.end());
    
    // 生成外围5个颜色
    _current_surround_colors.clear();
    
    // 保证有4个颜色与中心不同
    for (int i = 0; i < 4; i++) {
        if (i < static_cast<int>(available_indices.size())) {
            _current_surround_colors.push_back(COLOR_TABLE[available_indices[i]]);
        } else {
            // 如果可用颜色不够，使用默认颜色
            _current_surround_colors.push_back(Scalar(0, 0, 0));
        }
    }
    
    // 在目标位置插入与中心相同的颜色
    if (_current_target_idx < 4) {
        _current_surround_colors.insert(_current_surround_colors.begin() + _current_target_idx, 
                                       _current_center_color);
    } else {
        // 如果目标索引是4，放在最后
        _current_surround_colors.push_back(_current_center_color);
    }
}

// 获取训练用颜色组合
void TargetSim::get_training_colors(Scalar& center_color,
                                   vector<Scalar>& surround_colors,
                                   int& target_idx) {
    // 随机选择中心颜色
    int center_idx = rand() % COLOR_TABLE.size();
    center_color = COLOR_TABLE[center_idx];
    
    // 随机选择目标索引
    target_idx = rand() % 5;
    
    // 准备可用的颜色索引（排除中心颜色）
    vector<int> available_indices;
    for (size_t i = 0; i < COLOR_TABLE.size(); i++) {
        if (i != static_cast<size_t>(center_idx)) {
            available_indices.push_back(i);
        }
    }
    
    // 打乱可用颜色
    random_shuffle(available_indices.begin(), available_indices.end());
    
    // 清空并生成外围颜色
    surround_colors.clear();
    
    // 先添加4个不同颜色
    for (int i = 0; i < 4; i++) {
        if (i < static_cast<int>(available_indices.size())) {
            surround_colors.push_back(COLOR_TABLE[available_indices[i]]);
        } else {
            surround_colors.push_back(Scalar(0, 0, 0));
        }
    }
    
    // 在目标位置插入相同颜色
    if (target_idx < 4) {
        surround_colors.insert(surround_colors.begin() + target_idx, center_color);
    } else {
        surround_colors.push_back(center_color);
    }
}

// 生成五角星靶子图像
Mat TargetSim::generate_pentagon_frame(const Point2f& base_center,
                                      float rotation_angle,
                                      int target_idx) {
    // 如果未指定target_idx，使用当前颜色组合
    int actual_target_idx = (target_idx == -1) ? _current_target_idx : target_idx;
    
    // 计算布局
    auto centers = _calculate_pentagon_layout(base_center, rotation_angle);
    
    // 创建图像
    Mat image(_height, _width, CV_8UC3, _background);
    
    // 绘制靶子
    _draw_target(image, centers, _current_center_color, 
                _current_surround_colors, actual_target_idx);
    
    return image;
}

// 生成单色块目标图像
Mat TargetSim::generate_single_blob_frame(const Point2f& center, int size) {
    Mat image(_height, _width, CV_8UC3, _background);
    
    // 随机选择颜色
    Scalar color = COLOR_TABLE[rand() % COLOR_TABLE.size()];
    
    // 绘制单个色块
    circle(image, center, size, color, -1);
    
    // 更新中心坐标记录（仅记录这一个）
    _blob_centroids.clear();
    _blob_centroids.push_back(center);
    
    return image;
}

// 获取随机位置
Point2f TargetSim::get_random_position(float margin) {
    float x = margin + static_cast<float>(rand()) / RAND_MAX * (_width - 2 * margin);
    float y = margin + static_cast<float>(rand()) / RAND_MAX * (_height - 2 * margin);
    return Point2f(x, y);
}

// 计算五角星布局
vector<Point2f> TargetSim::_calculate_pentagon_layout(const Point2f& base_center,
                                                     float rotation_angle) {
    vector<Point2f> centers;
    
    // 中心色块
    centers.push_back(base_center);
    
    // 外围5个色块（半径100像素）
    float radius = 100.0f;
    
    for (int i = 0; i < 5; ++i) {
        float base_angle = i * (2 * M_PI / 5);
        float current_angle = base_angle + rotation_angle;
        
        centers.push_back(Point2f(
            base_center.x + radius * cos(current_angle),
            base_center.y + radius * sin(current_angle)
        ));
    }
    
    _blob_centroids = centers;
    return centers;
}

// 绘制靶子
void TargetSim::_draw_target(Mat& image, 
                           const vector<Point2f>& centers,
                           const Scalar& center_color,
                           const vector<Scalar>& surround_colors,
                           int target_idx) {
    int radius = 30;
    
    // 绘制中心色块
    circle(image, centers[0], radius, center_color, -1);
    
    // 绘制外围5个色块
    for (size_t i = 0; i < surround_colors.size(); ++i) {
        Scalar color = surround_colors[i];
        circle(image, centers[i + 1], radius, color, -1);
    }
}