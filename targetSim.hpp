#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

/**
 * @class TargetSim
 * @brief 靶子图像仿真器类
 * 
 * 用于生成包含6个色块（中心1个，外围5个）的仿真靶子图像。
 * 支持控制靶子位置、旋转角度、颜色布局，用于测试视觉追踪算法。
 */
class TargetSim {
public:
    /**
     * @brief 构造函数
     * @param width 画布宽度（像素）
     * @param height 画布高度（像素）
     * @param background 画布背景颜色（BGR格式）
     */
    TargetSim(int width = 640, int height = 480, 
              cv::Scalar background = cv::Scalar(255, 255, 255));
    
    /**
     * @brief 获取所有预定义颜色
     * @return 包含12个预定义颜色（BGR格式）的向量
     */
    std::vector<cv::Scalar> get_predefined_colors() const { return _predefined_colors; }

    /**
     * @brief 从预定义颜色中随机选择指定数量的颜色
     * @param count 要选择的颜色数量（不超过12）
     * @return 随机选择的颜色向量
     */
    std::vector<cv::Scalar> get_random_colors(int count = 5);

    /**
     * @brief 生成靶子图像（核心函数）
     * @param center_color 中心色块的颜色（BGR格式）
     * @param surround_colors 外围5个色块的颜色向量（BGR格式）
     * @param pattern 布局模式，当前支持 "pentagon"（五边形布局）
     * @param base_center 靶子中心在画布上的位置。默认值(-1,-1)表示使用画布中心
     * @param rotation_angle 外围色块围绕中心旋转的角度（弧度）。0表示不旋转
     * @param target_idx 外围色块中与中心同色的那个色块的索引（0-4）
     * @return 生成的靶子图像（cv::Mat）
     * 
     * @note 当 base_center 为默认值(-1,-1)时，会自动使用画布中心作为靶子中心。
     *       这是为了向后兼容旧调用方式。
     */
    cv::Mat generate_frame(const cv::Scalar& center_color,
                           const std::vector<cv::Scalar>& surround_colors,
                           const std::string& pattern = "pentagon",
                           const cv::Point2f& base_center = cv::Point2f(-1, -1),
                           float rotation_angle = 0.0f,
                           int target_idx = 0);

     /**
     * @brief 生成全随机的靶子图像（位置+旋转+颜色）
     * @param target_idx 目标色块索引
     * @return 生成的图像
     * 
     * @note 这是唯一保留的便捷函数，因为组合使用场景最多
     */
    cv::Mat generate_random_all_frame(int target_idx = 0);
    

    /**
     * @brief 获取最新生成的色块中心坐标
     * @return 包含6个色块中心坐标（中心+外围5个）的向量
     * 
     * @note 用于算法验证。生成图像后调用此函数可获得色块的真实位置（Ground Truth），
     *       与算法检测结果进行对比以评估精度。
     */
    std::vector<cv::Point2f> get_blob_centroids() const { return _blob_centroids; }

private:
    int _width;                         ///< 画布宽度（像素）
    int _height;                        ///< 画布高度（像素）
    cv::Scalar _background;             ///< 画布背景颜色（BGR）
    std::vector<cv::Point2f> _blob_centroids; ///< 最新生成的色块中心坐标
    std::vector<cv::Scalar> _predefined_colors; ///< 预定义的12种颜色（BGR格式）
    
    /**
     * @brief 计算靶子布局（核心几何计算）
     * @param pattern 布局模式
     * @param base_center 靶子中心位置
     * @param rotation_angle 旋转角度（弧度）
     * @return 6个色块的中心坐标向量（第一个为中心，后5个为外围）
     * 
     * @note 负责根据几何参数计算每个色块的实际像素位置。
     */
    std::vector<cv::Point2f> _calculate_layout(const std::string& pattern,
                                               const cv::Point2f& base_center,
                                               float rotation_angle);
    
    /**
     * @brief 生成随机位置（确保在画布内）
     * @return 随机生成的中心点坐标
     * 
     * @note 考虑靶子大小（半径约100像素），在画布边缘留出足够margin，
     *       确保生成的靶子完全在画面内。
     */
    cv::Point2f _get_random_position();
    
    /**
     * @brief 在图像上绘制靶子
     * @param image 目标图像（将被修改）
     * @param centers 6个色块的中心坐标
     * @param center_color 中心色块颜色
     * @param surround_colors 外围色块颜色向量
     * @param target_idx 目标色块索引（该外围色块使用中心颜色）
     * 
     * @note 实际绘制函数，根据坐标和颜色在图像上绘制圆形色块。
     */
    void _draw_target(cv::Mat& image, 
                      const std::vector<cv::Point2f>& centers,
                      const cv::Scalar& center_color,
                      const std::vector<cv::Scalar>& surround_colors,
                      int target_idx);
};