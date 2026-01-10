#ifndef TARGET_SIM_HPP
#define TARGET_SIM_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

/**
 * @class TargetSim
 * @brief 靶子图像仿真器 - 能生成二维靶子图像
 */
class TargetSim {
public:
    // 预定义的12种颜色（BGR格式）
    static const std::vector<cv::Scalar> COLOR_TABLE;
    
    /**
     * @brief 构造函数
     * @param width 生成图像的宽度（像素）
     * @param height 生成图像的高度（像素）
     * @param background 图像背景颜色（默认白色）
     */
    TargetSim(int width = 800, int height = 480, 
              cv::Scalar background = cv::Scalar(255, 255, 255));
    
    /**
     * @brief 生成五角星靶子图像
     * @param base_center 靶子中心位置
     * @param rotation_angle 旋转角度（弧度）
     * @param target_position 目标色块位置（与中心同色的外围色块）
     * @return 生成的靶子图像
     */
    cv::Mat generate_pentagon_frame(const cv::Point2f& base_center,
                                    float rotation_angle = 0.0f,
                                    cv::Point2f* target_position = nullptr);
    
    /**
     * @brief 生成单色块目标图像
     * @param center 色块中心位置
     * @param size 色块大小（半径）
     * @param use_random_color 是否使用随机颜色（默认false，使用上次颜色）
     * @param specified_color 指定颜色（可选，优先级最高）
     * @return 生成的图像
     */
    cv::Mat generate_single_blob_frame(const cv::Point2f& center,
                                       int size = 30,
                                       bool use_random_color = false,
                                       const cv::Scalar& specified_color = cv::Scalar(-1, -1, -1));

    /**
     * @brief 获取上一次生成的目标位置
     * @note 需要在调用 generate_pentagon_frame 后使用
     */
    cv::Point2f get_last_target_position() const { return _last_target_position; }
    
    /**
     * @brief 获取色块中心坐标（Ground Truth）
     * @return 包含6个坐标的向量（中心+外围5个）
     */
    std::vector<cv::Point2f> get_blob_centroids() const { return _blob_centroids; }
    
    /**
     * @brief 获取图像中心点坐标
     */
    cv::Point2f get_center() const { 
        return cv::Point2f(_width / 2.0f, _height / 2.0f); 
    }
    
    /**
     * @brief 获取图像宽度和高度
     */
    int get_width() const { return _width; }
    int get_height() const { return _height; }
    
    /**
     * @brief 获取随机位置
     * @param margin 边缘留白（默认120像素）
     */
    cv::Point2f get_random_position(float margin = 120.0f);

    /**
     * @brief 获取上一次使用的目标颜色
     */
    cv::Scalar get_last_target_color() const { return _last_target_color; }

    /**
     * @brief 强制重新生成颜色组合
     * @note 下次调用 generate_pentagon_frame 时会使用新颜色
     */
    void regenerate_colors() { _need_regenerate_colors = true; }

private:
    int _width;// 图像宽度
    int _height;// 图像高度
    cv::Scalar _background;// 图像背景颜色
    std::vector<cv::Point2f> _blob_centroids;// 色块中心坐标（Ground Truth）
    cv::Scalar _last_target_color;// 上一次使用的目标颜色
    cv::Point2f _last_target_position;// 上一次使用的目标位置
    
    // 当前的颜色组合
    cv::Scalar _center_color;
    std::vector<cv::Scalar> _surround_colors;
    
    // 状态跟踪变量 - 按初始化顺序排列
    cv::Point2f _current_center;
    float _current_rotation;
    int _target_index;  // 目标色块在外围中的索引
    bool _need_regenerate_colors;

    
    /**
     * @brief 计算五角星布局
     */
    std::vector<cv::Point2f> _calculate_pentagon_layout(const cv::Point2f& base_center,
                                                        float rotation_angle);
    
    /**
     * @brief 绘制靶子
     */
    void _draw_target(cv::Mat& image, 
                     const std::vector<cv::Point2f>& centers, float pixels_per_mm = 1.0f);
    
    /**
     * @brief 生成符合要求的颜色组合
     * @param target_index [输出] 目标色块在外围中的索引
     */
    void _generate_valid_color_combo(int& target_index);
};

#endif // TARGET_SIM_HPP