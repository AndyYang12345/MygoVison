#ifndef TARGET_TRACKER_HPP
#define TARGET_TRACKER_HPP
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include "TargetSim/TrainingFrameGenerator.hpp"

// ============ 算法核心参数配置 ============
// 色块筛选参数
#define SATURATION_THRESHOLD 50       // HSV饱和度阈值，过滤灰暗区域
#define MIN_BLOB_AREA 3000            // 最小色块面积（像素）
#define MAX_BLOB_AREA 8000            // 最大色块面积（像素）
#define MIN_CONTOUR_POINTS 5          // 有效轮廓最少点数

// 形状识别参数
#define CIRCULARITY_THRESHOLD 0.55f   // 圆形度阈值（1.0为完美圆）
#define MAX_ASPECT_RATIO 3.0f         // 正方形最大宽高比（容忍透视变形）
#define MIN_ASPECT_RATIO 0.8f         // 正方形最小宽高比

// 空间关系参数
#define MIN_DISTANCE_TO_CENTER 130.0f // 外围色块离中心的最小距离（像素）
#define MAX_DISTANCE_TO_CENTER 190.0f // 外围色块离中心的最大距离（像素）
#define EXPECTED_RADIUS 160.0f        // 期望的靶子半径（像素）
#define DISTANCE_TOLERANCE 50.0f      // 距离容忍范围

// 匹配阈值参数
#define MATCH_THRESHOLD 0.3f          // 综合匹配阈值（0.0-1.0）
#define CENTER_SCORE_THRESHOLD 0.4f   // 中心识别最低分数
#define DIRECTION_WEIGHT 0.4f         // 方向一致性权重
#define DISTANCE_WEIGHT 0.4f          // 距离评分权重
#define ASPECT_RATIO_WEIGHT 0.2f      // 宽高比评分权重
#define CIRCULARITY_WEIGHT 0.6f       // 圆形度权重（中心识别）
#define SURROUND_WEIGHT 0.4f          // 被围绕程度权重（中心识别）

// 颜色匹配参数
#define COLOR_SIMILARITY_THRESHOLD 50.0f // 颜色相似度阈值（BGR空间欧氏距离）
#define COLOR_CLUSTER_THRESHOLD 4.0f     // K-means聚类中心数量

// ============ 数据结构定义 ============
// 色块数据结构
struct ColorBlob {
    std::vector<cv::Point> contour;     // 轮廓点集
    cv::Rect bounding_rect;             // 外接矩形
    cv::Point2f center;                 // 中心点坐标
    cv::Scalar color_bgr;               // 平均BGR颜色值
    int color_label;                    // 聚类颜色标签
    float circularity;                  // 圆形度（0.0-1.0）
    float aspect_ratio;                 // 宽高比（宽度/高度）
    
    ColorBlob() : color_label(-1), circularity(0.0f), aspect_ratio(1.0f) {}
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
    
    // 颜色匹配
    float color_similarity_threshold = COLOR_SIMILARITY_THRESHOLD;
    int color_cluster_threshold = COLOR_CLUSTER_THRESHOLD;
    
    // 匹配阈值
    float match_threshold = MATCH_THRESHOLD;
    
    // 权重参数
    float direction_weight = DIRECTION_WEIGHT;
    float distance_weight = DISTANCE_WEIGHT;
    float aspect_ratio_weight = ASPECT_RATIO_WEIGHT;
    float circularity_weight = CIRCULARITY_WEIGHT;
    float surround_weight = SURROUND_WEIGHT;
};

class TargetTracker {
public:
    TargetTracker(){
        reset();
    }

    // 重置追踪器状态
    void reset() {
        total_frames_ = 0;
        successful_detections_ = 0;
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

private:
    // 配置参数
    TrackerConfig config_;

    // 检测准确率统计
    int total_frames_ = 0;
    int successful_detections_ = 0;
    
    // 内部辅助函数
    float calculate_color_distance(const cv::Scalar& color1, const cv::Scalar& color2) {
        float b_diff = color1[0] - color2[0];
        float g_diff = color1[1] - color2[1];
        float r_diff = color1[2] - color2[2];
        return sqrt(b_diff*b_diff + g_diff*g_diff + r_diff*r_diff);
    }

    void assign_color_labels_by_distance(std::vector<ColorBlob>& blobs);
};

#endif // TARGET_TRACKER_HPP