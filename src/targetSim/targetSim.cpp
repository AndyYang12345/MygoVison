#include "targetSim.hpp"
#include<opencv2/opencv.hpp>
#include <cmath>
using namespace cv;
using namespace std;

TargetSim::TargetSim(int width, int height, cv::Scalar background) 
    : _width(width), _height(height), _background(background) {
    cout << "Simulated Target Initialized" << endl;
    // 初始化随机种子
    srand(static_cast<unsigned int>(time(nullptr)));

    // 初始化12个预定义颜色（BGR格式）
    _predefined_colors = {
        cv::Scalar(116, 5, 202),    // 索引0
        cv::Scalar(167, 1, 98),     // 索引3
        cv::Scalar(7, 237, 19),     // 索引4
        cv::Scalar(23, 51, 215),    // 索引5
        cv::Scalar(241, 132, 251),  // 索引6
        cv::Scalar(17, 168, 214),   // 索引7
        cv::Scalar(135, 199, 246),  // 索引8
        cv::Scalar(221, 66, 76),    // 索引9
        cv::Scalar(216, 199, 167),  // 索引10
        cv::Scalar(85, 152, 55),    // 索引11
        cv::Scalar(57, 244, 231),   // 索引12
        cv::Scalar(23, 3, 23)       // 索引16
    };
}

// 修改1: generate_frame 增加位置和旋转参数
cv::Mat TargetSim::generate_frame(const cv::Scalar& center_color, 
                                  const std::vector<cv::Scalar>& surround_colors,
                                  const std::string& pattern,
                                  const cv::Point2f& base_center,   // 新增：中心位置
                                  float rotation_angle,             // 新增：旋转角度
                                  int target_idx) {
    cv::Mat image(_height, _width, CV_8UC3, _background);
    auto centers = _calculate_layout(pattern, base_center, rotation_angle); // 使用新接口
    _draw_target(image, centers, center_color, surround_colors, target_idx);
    return image;
}

cv::Mat TargetSim::generate_random_all_frame(int target_idx) {
    // 随机位置
    cv::Point2f base_center = _get_random_position();
    
    // 随机旋转角度
    float random_rotation = static_cast<float>(rand()) / RAND_MAX * 2 * M_PI;
    
    // 从预定义颜色中随机选择5个颜色
    std::vector<cv::Scalar> random_colors = get_random_colors(5);
    
    // 固定中心颜色为红色
    cv::Scalar center_color(0, 0, 255);
    
    // 调用核心函数
    return generate_frame(center_color, random_colors, "pentagon", 
                         base_center, random_rotation, target_idx);
}

std::vector<cv::Scalar> TargetSim::get_random_colors(int count) {
    std::vector<cv::Scalar> selected_colors;
    
    if (count > static_cast<int>(_predefined_colors.size())) {
        count = _predefined_colors.size();
        std::cout << "警告：请求颜色数量超过预定义颜色数量，使用最大数量" << std::endl;
    }
    
    // 创建所有颜色的索引（不使用std::iota）
    std::vector<int> indices;
    for (size_t i = 0; i < _predefined_colors.size(); ++i) {
        indices.push_back(static_cast<int>(i));
    }
    
    // 随机打乱索引（使用简单的手动打乱）
    for (size_t i = 0; i < indices.size(); ++i) {
        int j = rand() % indices.size();
        std::swap(indices[i], indices[j]);
    }
    
    // 选择前count个颜色
    for (int i = 0; i < count; ++i) {
        selected_colors.push_back(_predefined_colors[indices[i]]);
    }
    
    return selected_colors;
}

cv::Point2f TargetSim::get_random_position() {
    return _get_random_position();
}

// 私有辅助函数：生成随机位置
cv::Point2f TargetSim::_get_random_position() {
    float margin = 120.0f; // 留白，避免靶子边缘超出画面
    float x = margin + static_cast<float>(rand()) / RAND_MAX * (_width - 2*margin);
    float y = margin + static_cast<float>(rand()) / RAND_MAX * (_height - 2*margin);
    return cv::Point2f(x, y);
}

// 修改4: _calculate_layout 更新为新接口（已有，保持不变）
std::vector<cv::Point2f> TargetSim::_calculate_layout(const std::string& pattern, 
                                                      const cv::Point2f& base_center,
                                                      float rotation_angle) {
    std::vector<cv::Point2f> centers;
    
    centers.push_back(base_center);
    
    if (pattern == "pentagon") {
        float radius = 100.0f;
        
        for (int i = 0; i < 5; ++i) {
            float base_angle = i * (2 * M_PI / 5);
            float current_angle = base_angle + rotation_angle;
            
            centers.push_back(cv::Point2f(
                base_center.x + radius * std::cos(current_angle),
                base_center.y + radius * std::sin(current_angle)
            ));
        }
    }
    
    _blob_centroids = centers;
    return centers;
}

// _draw_target 保持不变
void TargetSim::_draw_target(cv::Mat& image, 
                             const std::vector<cv::Point2f>& centers,
                             const cv::Scalar& center_color,
                             const std::vector<cv::Scalar>& surround_colors,
                             int target_idx) {
    int radius = 30;
    cv::circle(image, centers[0], radius, center_color, -1);
    for (size_t i = 0; i < surround_colors.size(); ++i) {
        cv::Scalar color = (i == target_idx) ? center_color : surround_colors[i];
        cv::circle(image, centers[i + 1], radius, color, -1);
    }
}