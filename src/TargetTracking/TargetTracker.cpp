#include "TargetTracker.hpp"
/*
    该类对象接收来自 TrainingFrameGenerator 的训练帧，并实现目标跟踪功能，返回跟踪结果，包括位置和速度等信息。
    目前为止该类仅为框架，具体跟踪算法待实现。后续将集成卡尔曼滤波器等高级跟踪算法。
*/
using namespace cv;
using namespace std;

TargetTracker::TargetTracker() {
    // 构造函数 - 可初始化内部状态变量
}

TargetTracker::target_info TargetTracker::process_frame(
    const TrainingFrameGenerator::TrainingFrame& frame) {
    target_info result;
    // 简单示例：直接使用Ground Truth位置作为跟踪结果
    result.position = frame.target_position;
    result.velocity = frame.velocity;




    return result;
}