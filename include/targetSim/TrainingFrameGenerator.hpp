// TrainingFrameGenerator.hpp
#ifndef TRAINING_FRAME_GENERATOR_HPP
#define TRAINING_FRAME_GENERATOR_HPP

#include "targetSim.hpp"
#include <functional>

/**
 * @class TrainingFrameGenerator
 * @brief 训练帧生成器 - 专为PID调参和算法训练设计
 */
class TrainingFrameGenerator {
public:
    // 训练模式
    enum TrainingMode {
        MODE_PENTAGON_ROTATION,    // 五角星旋转模式
        MODE_LINEAR_MOVEMENT,      // 线性移动模式
        MODE_RANDOM_APPEARANCE     // 随机出现模式
    };
    
    /**
     * @brief 构造函数
     * @param width 图像宽度
     * @param height 图像高度
     * @param fps 帧率
     */
    TrainingFrameGenerator(int width = 800, int height = 600, float fps = 30.0f);
    
    /**
     * @brief 设置训练模式
     * @param mode 训练模式
     * @param param1 模式参数1
     * @param param2 模式参数2
     */
    void set_training_mode(TrainingMode mode, float param1 = 0.0f, float param2 = 0.0f);
    
    /**
     * @brief 获取下一训练帧
     * @param timestamp 时间戳（秒），如果为-1则使用内部时间
     * @return 训练帧和真实目标位置
     */
    struct TrainingFrame {
        cv::Mat frame;
        cv::Point2f target_position;   // 目标真实位置（像素）
        float timestamp;
        int target_index;              // 目标色块索引
    };
    
    TrainingFrame get_next_frame(float timestamp = -1.0f);
    
    /**
     * @brief 重置生成器
     */
    void reset();
    
    /**
     * @brief 获取当前时间
     */
    float get_current_time() const { return _current_time; }
    
    /**
     * @brief 获取当前模式
     */
    TrainingMode get_current_mode() const { return _current_mode; }

private:
    TargetSim _target_sim;
    float _fps;
    float _current_time;
    TrainingMode _current_mode;
    
    // 模式参数
    float _param1, _param2;
    
    // 内部状态
    cv::Point2f _current_position;
    float _last_appear_time;
    cv::Point2f _last_random_position;
    
    // 模式处理函数
    TrainingFrame _generate_pentagon_rotation(float timestamp);
    TrainingFrame _generate_linear_movement(float timestamp);
    TrainingFrame _generate_random_appearance(float timestamp);
    
    // 更新位置函数
    cv::Point2f _update_position_pentagon(float timestamp);
    cv::Point2f _update_position_linear(float timestamp);
    cv::Point2f _update_position_random(float timestamp);
};

#endif // TRAINING_FRAME_GENERATOR_HPP