#ifndef TARGET_TRACKER_HPP
#define TARGET_TRACKER_HPP
#include <opencv2/opencv.hpp>
#include <vector>
#include "TargetSim/TrainingFrameGenerator.hpp"

class TargetTracker {
public:
    /**
     * @brief 构造函数
     */
    TargetTracker();

    struct target_info
    {
        cv::Point2f position;
        cv::Point2f velocity;
    };
    
    
    /**
     * @brief 处理训练帧，返回跟踪结果
     * @param frame 训练帧
     * @return 目标位置和速度
     */
    target_info process_frame(
        const TrainingFrameGenerator::TrainingFrame& frame);

private:
    // 内部状态变量（如需要）
};

#endif // TARGET_TRACKER_HPP