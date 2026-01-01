// targetSim.hpp
#ifndef TARGET_SIM_HPP
#define TARGET_SIM_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

/**
 * @class TargetSim
 * @brief 靶子图像仿真器（简化版）- 专为视觉云台追踪训练设计
 */
class TargetSim {
public:
    // 预定义的12种颜色（BGR格式）
    static const std::vector<cv::Scalar> COLOR_TABLE;
    
    TargetSim(int width = 800, int height = 480, 
              cv::Scalar background = cv::Scalar(255, 255, 255));
    
    /**
     * @brief 生成五角星靶子图像
     * @param base_center 靶子中心位置
     * @param rotation_angle 旋转角度（弧度）
     * @param target_idx 目标色块索引（0-4），默认随机
     * @return 生成的靶子图像
     */
    cv::Mat generate_pentagon_frame(const cv::Point2f& base_center,
                                    float rotation_angle = 0.0f,
                                    int target_idx = -1);
    
    /**
     * @brief 生成单色块目标图像
     * @param center 色块中心位置
     * @param size 色块大小（半径）
     * @return 生成的图像
     */
    cv::Mat generate_single_blob_frame(const cv::Point2f& center,
                                       int size = 30);
    
    /**
     * @brief 获取训练用颜色组合
     * @param center_color [输出] 中心颜色
     * @param surround_colors [输出] 外围5个颜色
     * @param target_idx [输出] 目标色块索引
     * @note 保证外围有且只有一个颜色与中心颜色相同
     */
    static void get_training_colors(cv::Scalar& center_color,
                                   std::vector<cv::Scalar>& surround_colors,
                                   int& target_idx);
    
    /**
     * @brief 获取色块中心坐标（Ground Truth）
     * @return 包含6个坐标的向量（中心+外围5个）
     */
    std::vector<cv::Point2f> get_blob_centroids() const { return _blob_centroids; }
    
    cv::Point2f get_center() const { 
        return cv::Point2f(_width / 2.0f, _height / 2.0f); 
    }
    
    int get_width() const { return _width; }
    int get_height() const { return _height; }
    
    cv::Point2f get_random_position(float margin = 120.0f);

private:
    int _width;
    int _height;
    cv::Scalar _background;
    std::vector<cv::Point2f> _blob_centroids;
    
    cv::Scalar _current_center_color;
    std::vector<cv::Scalar> _current_surround_colors;
    int _current_target_idx;
    
    std::vector<cv::Point2f> _calculate_pentagon_layout(const cv::Point2f& base_center,
                                                        float rotation_angle);
    
    void _draw_target(cv::Mat& image, 
                     const std::vector<cv::Point2f>& centers,
                     const cv::Scalar& center_color,
                     const std::vector<cv::Scalar>& surround_colors,
                     int target_idx);
    
    /**
     * @brief 生成符合要求的颜色组合
     */
    void _generate_valid_color_combo();
};

#endif // TARGET_SIM_HPP