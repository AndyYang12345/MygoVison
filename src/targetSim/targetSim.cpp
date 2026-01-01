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
    : _width(width), _height(height), _background(background),
      _current_target_color(-1, -1, -1) {
    srand(static_cast<unsigned int>(time(nullptr)));
    _generate_valid_color_combo();
}

// 生成五角星靶子图像
// targetSim.cpp
Mat TargetSim::generate_pentagon_frame(const Point2f& base_center,
                                      float rotation_angle,
                                      int target_idx) {  // 新增参数
    // 计算布局
    auto centers = _calculate_pentagon_layout(base_center, rotation_angle);
    
    // 创建图像
    Mat image(_height, _width, CV_8UC3, _background);
    
    // 绘制靶子，传入 target_idx
    _draw_target(image, centers, _current_center_color, 
                _current_surround_colors, target_idx);
    
    // 保存当前目标索引
    _current_target_idx = target_idx;
    
    return image;
}

cv::Point2f TargetSim::get_target_position() const {
    // 检查是否有足够的数据
    if (_blob_centroids.size() > 1 && _current_target_idx >= 0) {
        // 根据当前目标索引返回正确的外围色块
        // 注意：_blob_centroids[0] 是中心，外围从索引1开始
        int target_centroid_idx = _current_target_idx + 1;
        if (target_centroid_idx < static_cast<int>(_blob_centroids.size())) {
            return _blob_centroids[target_centroid_idx];
        }
    }
    return cv::Point2f(-1, -1);  // 无效位置
}

// 生成单色块目标图像
Mat TargetSim::generate_single_blob_frame(const Point2f& center,
                                         int size,
                                         bool use_random_color,
                                         const Scalar& specified_color) {
    Mat image(_height, _width, CV_8UC3, _background);
    
    Scalar color;
    
    // 颜色选择优先级：指定颜色 > 随机颜色 > 保持上次颜色
    if (specified_color[0] >= 0) {  // 有效的BGR值
        color = specified_color;
    } else if (use_random_color) {
        // 随机选择颜色
        color = COLOR_TABLE[rand() % COLOR_TABLE.size()];
    } else {
        // 使用上次颜色，如果还没有则随机选择一个
        if (_current_target_color[0] < 0) {  // 还没有设置颜色
            color = COLOR_TABLE[rand() % COLOR_TABLE.size()];
        } else {
            color = _current_target_color;
        }
    }
    
    // 保存当前颜色
    _current_target_color = color;
    
    // 绘制单个色块
    circle(image, center, size, color, -1);
    
    // 可选：添加边框提高可见性
    circle(image, center, size + 3, Scalar(0, 0, 0), 2);
    
    // 更新中心坐标记录
    _blob_centroids.clear();
    _blob_centroids.push_back(center);
    
    return image;
}

// 获取训练用颜色组合（静态方法）
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
    
    // 修复：使用 std::shuffle 替代 random_shuffle
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(available_indices.begin(), available_indices.end(), g);
    // 删除或注释掉原来的：random_shuffle(available_indices.begin(), available_indices.end());
    
    // 清空并生成外围颜色
    surround_colors.clear();
    
    // 先添加4个不同颜色
    for (int i = 0; i < 4; i++) {
        if (i < static_cast<int>(available_indices.size())) {
            surround_colors.push_back(COLOR_TABLE[available_indices[i]]);
        } else {
            surround_colors.push_back(Scalar(0, 0, 0)); // 备用颜色
        }
    }
    
    // 在目标位置插入相同颜色
    if (target_idx < 4) {
        surround_colors.insert(surround_colors.begin() + target_idx, center_color);
    } else {
        surround_colors.push_back(center_color);
    }
}

// 获取随机位置
Point2f TargetSim::get_random_position(float margin) {
    float x = margin + static_cast<float>(rand()) / RAND_MAX * (_width - 2 * margin);
    float y = margin + static_cast<float>(rand()) / RAND_MAX * (_height - 2 * margin);
    return Point2f(x, y);
}

// 私有方法：生成符合要求的颜色组合
void TargetSim::_generate_valid_color_combo() {
    // 随机选择中心颜色
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
    
    // 修复：使用 std::shuffle 替代 random_shuffle
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(available_indices.begin(), available_indices.end(), g);
    // 删除或注释掉原来的：random_shuffle(available_indices.begin(), available_indices.end());
    
    // 生成外围5个颜色
    _current_surround_colors.clear();
    
    // 保证有4个颜色与中心不同
    for (int i = 0; i < 4; i++) {
        if (i < static_cast<int>(available_indices.size())) {
            _current_surround_colors.push_back(COLOR_TABLE[available_indices[i]]);
        } else {
            // 如果可用颜色不够，使用备用颜色
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


// 私有方法：计算五角星布局
vector<Point2f> TargetSim::_calculate_pentagon_layout(const Point2f& base_center,
                                                     float rotation_angle) {
    vector<Point2f> centers;
    
    // 中心色块
    centers.push_back(base_center);
    
    // 外围5个色块（半径100像素）
    float radius = 100.0f;
    
    for (int i = 0; i < 5; ++i) {
    // 关键：确保使用正确的起始角度和方向
    // 如果要让索引0对应"正上方"或特定位置，需要调整
    float base_angle = i * (2 * M_PI / 5); 
    
    // 如果需要从特定位置开始，可以添加偏移
    // 例如：float base_angle = i * (2 * M_PI / 5) + angle_offset;
    
    float current_angle = base_angle + rotation_angle;
    
    centers.push_back(Point2f(
        base_center.x + radius * cos(current_angle),
        base_center.y + radius * sin(current_angle)
    ));
    
    // 调试输出
    std::cout << "外围色块 " << i << ": 角度=" << (current_angle * 180 / M_PI) 
              << "°, 位置=(" << centers.back().x << ", " << centers.back().y << ")" << std::endl;
    }   
    
    _blob_centroids = centers;
    return centers;
}

// 私有方法：绘制靶子
void TargetSim::_draw_target(Mat& image, 
                           const vector<Point2f>& centers,
                           const Scalar& center_color,
                           const vector<Scalar>& surround_colors,
                           int target_idx) {  // target_idx 是关键参数！
    int radius = 30;
    
    // 绘制中心色块
    circle(image, centers[0], radius, center_color, -1);
    
    // 绘制外围5个色块
    for (size_t i = 0; i < surround_colors.size(); ++i) {
        Scalar color = surround_colors[i];
        circle(image, centers[i + 1], radius, color, -1);
        
        // 如果是目标色块，添加特殊标记（可选）
        if (static_cast<int>(i) == target_idx) {
            // 在目标色块上画一个外圈标记
            circle(image, centers[i + 1], radius + 5, Scalar(0, 255, 255), 2);
        }
    }
    
    // 保存色块中心坐标和目标索引
    _blob_centroids = centers;
    _current_target_idx = target_idx;  // 保存目标索引
}