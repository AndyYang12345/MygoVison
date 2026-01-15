#ifndef TARGET_TRACKER_HPP
#define TARGET_TRACKER_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include <numeric>

// ============ 算法核心参数配置 ============
// 色块筛选参数
#define SATURATION_THRESHOLD 50       // HSV饱和度阈值，过滤灰暗区域
#define MIN_BLOB_AREA 3000            // 最小色块面积（像素）
#define MAX_BLOB_AREA 8000            // 最大色块面积（像素）

// 形状识别参数
#define CIRCULARITY_THRESHOLD 0.55f   // 圆形度阈值（1.0为完美圆）
#define MAX_ASPECT_RATIO 3.0f         // 正方形最大宽高比（容忍透视变形）

// 空间关系参数
#define MIN_DISTANCE_TO_CENTER 130.0f // 外围色块离中心的最小距离（像素）
#define MAX_DISTANCE_TO_CENTER 190.0f // 外围色块离中心的最大距离（像素）
#define EXPECTED_RADIUS 160.0f        // 期望的靶子半径（像素）

// 匹配阈值参数
#define MATCH_THRESHOLD 0.4f          // 综合匹配阈值（0.0-1.0）
#define DIRECTION_WEIGHT 0.4f         // 方向一致性权重
#define DISTANCE_WEIGHT 0.4f          // 距离评分权重
#define ASPECT_RATIO_WEIGHT 0.2f      // 宽高比评分权重
#define CIRCULARITY_WEIGHT 0.6f       // 圆形度权重（中心识别）
#define SURROUND_WEIGHT 0.4f          // 被围绕程度权重（中心识别）

// HSV颜色相似性参数
#define HUE_SIMILARITY_THRESHOLD 5.0f    // 色调相似度阈值（度）
#define VALUE_MIN_THRESHOLD 15.0f         // 亮度最小值阈值（避免黑色干扰）
#define BLACK_VALUE_THRESHOLD 30.0f       // 黑色亮度阈值
#define BLACK_SATURATION_THRESHOLD 50.0f  // 黑色饱和度阈值

// ============ 数据结构定义 ============
// 色块数据结构
struct ColorBlob {
    std::vector<cv::Point> contour;     // 轮廓点集
    cv::Rect bounding_rect;             // 外接矩形
    cv::Point2f center;                 // 中心点坐标
    cv::Scalar color_bgr;               // 平均BGR颜色值
    cv::Scalar color_hsv;               // 平均HSV颜色值
    int color_label;                    // 聚类颜色标签
    float circularity;                  // 圆形度（0.0-1.0）
    float aspect_ratio;                 // 宽高比（宽度/高度）
    bool is_black;                      // 是否为黑色
    
    ColorBlob() : color_label(-1), circularity(0.0f), aspect_ratio(1.0f), is_black(false) {}
};

// 目标追踪结果结构
struct target_info {
    bool found;                         // 是否成功识别目标
    cv::Point2f target_center;          // 目标方块中心（像素坐标）
    cv::Point2f board_center;           // 靶子中心（像素坐标）
    float distance;                     // 目标到中心的距离（像素）
    float angle;                        // 目标相对中心的角度（度）
    int center_color_label;             // 中心颜色标签（调试用）
    
    target_info() : found(false), distance(0.0f), angle(0.0f), center_color_label(-1) {}
};

// 参数配置结构体
struct TrackerConfig {
    // 色块筛选
    int saturation_threshold = SATURATION_THRESHOLD;
    int min_blob_area = MIN_BLOB_AREA;
    int max_blob_area = MAX_BLOB_AREA;
    
    // 形状识别
    float circularity_threshold = CIRCULARITY_THRESHOLD;
    float max_aspect_ratio = MAX_ASPECT_RATIO;
    
    // 空间关系
    float min_distance_to_center = MIN_DISTANCE_TO_CENTER;
    float max_distance_to_center = MAX_DISTANCE_TO_CENTER;
    float expected_radius = EXPECTED_RADIUS;
    
    // 匹配阈值
    float match_threshold = MATCH_THRESHOLD;
    
    // 权重参数
    float direction_weight = DIRECTION_WEIGHT;
    float distance_weight = DISTANCE_WEIGHT;
    float aspect_ratio_weight = ASPECT_RATIO_WEIGHT;
    float circularity_weight = CIRCULARITY_WEIGHT;
    float surround_weight = SURROUND_WEIGHT;
    
    // HSV颜色参数
    float hue_similarity_threshold = HUE_SIMILARITY_THRESHOLD;
    float value_min_threshold = VALUE_MIN_THRESHOLD;
    float black_value_threshold = BLACK_VALUE_THRESHOLD;
    float black_saturation_threshold = BLACK_SATURATION_THRESHOLD;
    
    // 保持向后兼容性（如果测试代码引用了旧的参数名）
    float color_similarity_threshold = 50.0f; // 旧参数，保持兼容
};

class TargetTracker {
public:
    TargetTracker() : 
        total_frames_(0),
        successful_detections_(0),
        last_target_angle_(0.0f),
        last_angle_valid_(false),
        consecutive_success_(0),
        consecutive_failures_(0),
        angle_prediction_threshold_(3.0f),
        use_angle_constraint_(true),        // 默认启用角度约束
        dark_brightness_threshold_(50),     // 默认BGR亮度阈值50
        debug_mode_(false)                  // 默认关闭调试模式
    {
        config_ = get_config();
    }

    // 设置调试模式
    void set_debug_mode(bool enabled) {
        debug_mode_ = enabled;
    }

    /**
     * @brief 设置追踪器参数
     * @param config 新的参数配置
     */
    void set_config(const TrackerConfig& config) {
        config_ = config;
    }
    
    /**
     * @brief 获取当前参数配置
     * @return 当前参数配置
     */
    TrackerConfig get_config() const {
        return config_;
    }
    
    /**
     * @brief 处理训练帧，返回跟踪结果
     * @param frame 训练帧
     * @return 目标位置和速度
     */
    target_info process_frame(const cv::Mat& frame);

    /**
     * @brief 获取检测统计信息
     * @return 成功检测率
     */
    float get_success_rate() const {
        if (total_frames_ == 0) return 0.0f;
        return static_cast<float>(successful_detections_) / total_frames_;
    }

    // 添加设置先验参数的方法
    void set_angle_prediction_threshold(float threshold) {
        angle_prediction_threshold_ = threshold;
    }
    
    float get_angle_prediction_threshold() const {
        return angle_prediction_threshold_;
    }
    
    void reset_prior_info() {
        last_angle_valid_ = false;
        consecutive_success_ = 0;
        consecutive_failures_ = 0;
        angle_history_.clear();
        time_history_.clear();
    }

    // 添加设置方法
    void set_dark_brightness_threshold(int threshold) { 
        dark_brightness_threshold_ = threshold; 
    }
    
    int get_dark_brightness_threshold() const { 
        return dark_brightness_threshold_; 
    }

    void enable_angle_constraint(bool enable) {
        use_angle_constraint_ = enable;
    }

private:
    // 统计信息
    int total_frames_;
    int successful_detections_;
    
    // 配置
    TrackerConfig config_;
    
    // 先验角度信息
    float last_target_angle_;          // 上一次成功识别的目标角度
    bool last_angle_valid_;            // 先验角度是否有效
    int consecutive_success_;          // 连续成功次数
    int consecutive_failures_;         // 连续失败次数
    float angle_prediction_threshold_; // 角度预测阈值（度）
    bool use_angle_constraint_;        // 是否启用角度约束
    
    // 暗色检测阈值
    int dark_brightness_threshold_;    // BGR亮度阈值
    
    // 调试模式
    bool debug_mode_;                  // 调试模式标志
    
    // 历史记录
    std::deque<float> angle_history_;  // 角度历史记录
    std::deque<float> time_history_;   // 时间历史记录
    // 辅助函数
    float normalize_angle(float angle); // 将角度标准化到[0, 360)
    float angle_difference(float a, float b); // 计算两个角度之间的最小差异
    
    // 内部辅助函数
    float calculate_color_distance_hsv(const cv::Scalar& hsv1, const cv::Scalar& hsv2);
    bool is_black_color_hsv(const cv::Scalar& hsv_color);
    cv::Scalar convert_bgr_to_hsv(const cv::Scalar& bgr_color);
    void assign_color_labels_by_hsv(std::vector<ColorBlob>& blobs);
    float calculate_surround_score(const std::vector<ColorBlob>& blobs, size_t center_idx);
    float calculate_color_distance_bgr(const cv::Scalar& bgr1, const cv::Scalar& bgr2);
    
    // 添加判断是否为暗色函数
    bool is_dark_color_bgr(const cv::Scalar& bgr_color);
    
    // 添加使用BGR空间的颜色相似性检查
    bool check_color_similarity_bgr(const cv::Scalar& bgr1, const cv::Scalar& bgr2, float threshold);
    
    // 添加混合颜色空间的距离计算
    float calculate_color_distance_mixed(const cv::Scalar& color1, const cv::Scalar& color2, 
                                        bool use_bgr_space);
};

#endif // TARGET_TRACKER_HPP