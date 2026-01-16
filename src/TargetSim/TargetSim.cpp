#include "TargetSim/TargetSim.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <random>
#include <chrono>

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
      _last_target_color(-1, -1, -1), _last_target_position(-1, -1),
      _current_center(-1, -1), _current_rotation(0.0f),
      _target_index(-1), _need_regenerate_colors(true) {
    unsigned seed = chrono::system_clock::now().time_since_epoch().count();
    srand(seed);
}

// 生成五角星靶子图像
Mat TargetSim::generate_pentagon_frame(const Point2f& base_center,
                                      float rotation_angle,
                                      cv::Point2f* target_position) {
    // 如果传入的中心点是特殊值（如 (-1, -1)），则使用随机位置
    Point2f actual_center = base_center;
    bool is_random_position = false;
    
    if (base_center.x < 0 && base_center.y < 0) {
        actual_center = get_random_position(120.0f);
        is_random_position = true;
    }
    
    // 计算布局
    auto centers = _calculate_pentagon_layout(actual_center, rotation_angle);
    
    // 生成颜色组合（如果需要）
    if (_need_regenerate_colors || is_random_position) {
        int target_index;
        _generate_valid_color_combo(target_index);
        _need_regenerate_colors = false;
        _target_index = target_index;
    }
    
    // 计算目标位置
    if (_target_index >= 0 && centers.size() > static_cast<size_t>(_target_index + 1)) {
        _last_target_position = centers[_target_index + 1];
        _last_target_color = _center_color;
        
        // 添加实例标识
        static int instance_id = 0;
        if (is_random_position) instance_id++;
        
        // std::cout << "[DEBUG] Instance " << instance_id << " - ";
        // std::cout << (is_random_position ? "RANDOM pentagon" : "MAIN pentagon") << std::endl;
        // std::cout << "  Center: (" << actual_center.x << ", " << actual_center.y << ")" << std::endl;
        // std::cout << "  Rotation: " << (rotation_angle * 180 / M_PI) << "°" << std::endl;
        // std::cout << "  Target index: " << _target_index << std::endl;
        // std::cout << "  Target position: (" << _last_target_position.x << ", " << _last_target_position.y << ")" << std::endl;
    }
    
    // 如果调用者需要目标位置，则输出
    if (target_position != nullptr) {
        *target_position = _last_target_position;
    }
    
    // 创建图像
    Mat image(_height, _width, CV_8UC3, _background);
    
    // 绘制靶子
    _draw_target(image, centers);
    
    return image;
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
        if (_last_target_color[0] < 0) {  // 还没有设置颜色
            color = COLOR_TABLE[rand() % COLOR_TABLE.size()];
        } else {
            color = _last_target_color;
        }
    }
    
    // 保存当前颜色
    _last_target_color = color;
    _last_target_position = center;
    
    // 绘制单个色块
    circle(image, center, size, color, -1);
    
    // 更新中心坐标记录
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

// 私有方法：生成符合要求的颜色组合
void TargetSim::_generate_valid_color_combo(int& target_index) {
    // 随机选择中心颜色
    int center_idx = rand() % COLOR_TABLE.size();
    _center_color = COLOR_TABLE[center_idx];
    
    // 随机选择目标索引（0-4）
    target_index = rand() % 5;
    
    // 准备可用的颜色索引（排除中心颜色）
    vector<int> available_indices;
    for (size_t i = 0; i < COLOR_TABLE.size(); i++) {
        if (i != static_cast<size_t>(center_idx)) {
            available_indices.push_back(i);
        }
    }
    

    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(available_indices.begin(), available_indices.end(), g);
    
    // 生成外围5个颜色
    _surround_colors.clear();
    
    // 保证有4个颜色与中心不同
    for (int i = 0; i < 4; i++) {
        if (i < static_cast<int>(available_indices.size())) {
            _surround_colors.push_back(COLOR_TABLE[available_indices[i]]);
        } else {
            // 如果可用颜色不够，使用备用颜色
            _surround_colors.push_back(Scalar(0, 0, 0));
        }
    }
    
    // 在目标位置插入与中心相同的颜色
    if (target_index < 4) {
        _surround_colors.insert(_surround_colors.begin() + target_index, 
                               _center_color);
    } else {
        // 如果目标索引是4，放在最后
        _surround_colors.push_back(_center_color);
    }
}

// 私有方法：计算五角星布局
vector<Point2f> TargetSim::_calculate_pentagon_layout(const Point2f& base_center,
                                                     float rotation_angle) {
    vector<Point2f> centers;
    
    // 中心色块
    centers.push_back(base_center);

    // 外围5个色块（半径160像素）
    float radius = 160.0f;
    
    // 重要：确保正确使用旋转角度
    for (int i = 0; i < 5; ++i) {
        // 计算每个色块的基本角度（均匀分布）
        float base_angle = i * (2 * M_PI / 5);
        
        // 应用旋转角度
        float current_angle = base_angle + rotation_angle;
        
        // 计算位置
        float x = base_center.x + radius * cos(current_angle);
        float y = base_center.y + radius * sin(current_angle);
        
        centers.push_back(Point2f(x, y));
        
        // // 调试输出前几次调用
        // static int call_count = 0;
        // if (call_count < 5) {
        //     std::cout << "[DEBUG] Layout calculation:" << std::endl;
        //     std::cout << "  i=" << i << ", base_angle=" << (base_angle * 180 / M_PI) << "°" 
        //               << ", rotation=" << (rotation_angle * 180 / M_PI) << "°"
        //               << ", final_angle=" << (current_angle * 180 / M_PI) << "°" << std::endl;
        //     std::cout << "  Position: (" << x << ", " << y << ")" << std::endl;
        //     call_count++;
        // }
    }   
    
    _blob_centroids = centers;
    return centers;
}

// 私有方法：绘制靶子
void TargetSim::_draw_target(Mat& image, const vector<Point2f>& centers,
                           float pixels_per_mm) {
    // 根据比例因子计算像素尺寸
    int circle_radius_px = static_cast<int>(40.0f * pixels_per_mm);      // 80mm
    int square_size_px = static_cast<int>(80.0f * pixels_per_mm);        // 80mm边长
    
    // 1. 绘制中心圆
    circle(image, centers[0], circle_radius_px, _center_color, -1);
    
    // 2. 绘制外围正方形（共5个）
    for (size_t i = 0; i < _surround_colors.size() && i + 1 < centers.size(); ++i) {
        Point2f square_center = centers[i + 1];
        Scalar color = _surround_colors[i];
        
        // 计算指向中心的旋转角度
        Point2f direction = centers[0] - square_center;
        double angle_deg = atan2(direction.y, direction.x) * 180.0 / CV_PI;
        
        // 重要：正方形对角线指向中心，所以需要旋转-45度
        // 因为正方形的默认方向是对角线水平/垂直的
        double rotation_angle = angle_deg - 45.0;
        
        // 使用RotatedRect（更可靠）
        RotatedRect rect(square_center, 
                        Size2f(square_size_px, square_size_px),
                        rotation_angle);
        
        // 获取顶点并绘制
        Point2f vertices[4];
        rect.points(vertices);
        
        // 转换为整数坐标
        vector<Point> int_vertices;
        for (int j = 0; j < 4; j++) {
            // 确保坐标在图像范围内
            int x = cvRound(vertices[j].x);
            int y = cvRound(vertices[j].y);
            x = max(0, min(x, image.cols - 1));
            y = max(0, min(y, image.rows - 1));
            int_vertices.push_back(Point(x, y));
        }
        fillConvexPoly(image, int_vertices, color, LINE_AA);
    }
}