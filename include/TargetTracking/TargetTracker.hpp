#ifndef TARGET_TRACKER_HPP
#define TARGET_TRACKER_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

// 配置结构体
struct TrackerConfig {
    // 预处理参数
    int blur_size = 5;
    double blur_sigma = 1.5;
    
    // 掩码参数
    int saturation_threshold = 50;
    int value_threshold = 50;
    int dark_brightness_threshold = 100;
    
    // 色块过滤参数
    int min_blob_area = 500;
    int max_blob_area = 10000;
    float min_circularity = 0.6f;
    
    // 中心检测参数
    float min_distance_to_center = 50.0f;
    float max_distance_to_center = 250.0f;
    
    // 调试选项
    bool show_debug_windows = false;
    bool print_debug_info = false;
};

// 色块信息结构体
struct ColorBlob {
    cv::Rect bounding_rect;      // 边界框
    cv::Point2f center;          // 中心点
    double area;                 // 面积
    double circularity;          // 圆形度
    cv::Scalar mean_color_bgr;   // 平均颜色 (BGR)
    cv::Scalar mean_color_hsv;   // 平均颜色 (HSV)
    bool is_dark;                // 是否为深色
};

// 跟踪结果结构体
struct TargetInfo {
    bool found = false;               // 是否找到目标
    cv::Point2f board_center;         // 标靶板中心
    cv::Point2f target_center;        // 目标色块中心
    float distance = 0.0f;            // 距离（像素）
    float angle = 0.0f;               // 角度（度）
};

// 主跟踪器类
class TargetTracker {
public:
    TargetTracker();
    
    // 配置管理
    void set_config(const TrackerConfig& config);
    TrackerConfig get_config() const;
    void enable_debug(bool enabled);
    
    // 主处理函数
    TargetInfo process_frame(const cv::Mat& frame);
    
    // 统计分析
    void reset_statistics();
    void print_statistics() const;
    
private:
    // 核心处理函数
    std::vector<ColorBlob> extract_color_blobs(const cv::Mat& frame, cv::Mat& debug_mask);
    ColorBlob* find_center_blob(std::vector<ColorBlob>& blobs);
    ColorBlob* find_matching_target(const std::vector<ColorBlob>& blobs, const ColorBlob& center_blob);
    
    // 颜色匹配函数
    double calculate_color_similarity(const cv::Scalar& color1, const cv::Scalar& color2, bool center_is_dark);
    
    // 辅助函数
    double calculate_circularity(const std::vector<cv::Point>& contour);
    bool is_valid_surrounding_blob(const ColorBlob& blob, const ColorBlob& center);
    cv::Scalar bgr_to_hsv(const cv::Scalar& bgr);
    cv::Scalar bgr_to_lab(const cv::Scalar& bgr);
    bool is_dark_color(const cv::Scalar& bgr, int threshold);
    // 调试功能
    void draw_debug_info(cv::Mat& frame, 
                        const std::vector<ColorBlob>& blobs,
                        const ColorBlob* center,
                        const ColorBlob* target);
    
    // 成员变量
    TrackerConfig config_;
    cv::Size frame_size_;
    int frames_processed_;
    int successful_tracks_;
    struct ColorSimilarity {
        double bgr_sim;
        double hsv_sim;
        double lab_sim;
        double combined_sim;
    };
    ColorSimilarity calculate_multi_space_similarity(const cv::Scalar& color1, const cv::Scalar& color2, bool center_is_dark);
    int classify_color_type(const cv::Scalar& hsv);
};

#endif // TARGET_TRACKER_HPP