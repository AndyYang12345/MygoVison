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

// 修改2: generate_random_position_frame 使用新的_calculate_layout
cv::Mat TargetSim::generate_random_position_frame(const cv::Scalar& center_color,
                                                  const std::vector<cv::Scalar>& surround_colors,
                                                  int target_idx) {
    cv::Mat image(_height, _width, CV_8UC3, _background);
    
    // 随机位置（不带旋转）
    cv::Point2f base_center = _get_random_position();
    auto centers = _calculate_layout("pentagon", base_center, 0.0f); // 明确传入0.0f旋转
    _draw_target(image, centers, center_color, surround_colors, target_idx);
    
    return image;
}

// 修改3: generate_random_pose_frame 使用新的_calculate_layout
cv::Mat TargetSim::generate_random_pose_frame(const cv::Scalar& center_color,
                                              const std::vector<cv::Scalar>& surround_colors,
                                              int target_idx) {
    cv::Mat image(_height, _width, CV_8UC3, _background);
    
    // 随机位置
    cv::Point2f base_center = _get_random_position();
    
    // 随机旋转角度
    float random_rotation = static_cast<float>(rand()) / RAND_MAX * 2 * M_PI;
    
    auto centers = _calculate_layout("pentagon", base_center, random_rotation);
    _draw_target(image, centers, center_color, surround_colors, target_idx);
    
    return image;
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