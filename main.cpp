#include "targetSim.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    TargetSim sim(800, 600, cv::Scalar(220, 220, 220));
    
    // 示例1：生成固定位置、固定颜色的靶子
    std::vector<cv::Scalar> colors1 = {
        cv::Scalar(0, 255, 0),     // 绿
        cv::Scalar(255, 0, 0),     // 蓝
        cv::Scalar(255, 255, 0),   // 青
        cv::Scalar(255, 0, 255),   // 紫
        cv::Scalar(0, 255, 255)    // 黄
    };
    
    // 创建窗口
    cv::namedWindow("Fixed Position", cv::WINDOW_NORMAL);
    cv::namedWindow("Random Position", cv::WINDOW_NORMAL);
    cv::namedWindow("Random Pose", cv::WINDOW_NORMAL);
    cv::namedWindow("Random Colors", cv::WINDOW_NORMAL);
    cv::namedWindow("Fully Random", cv::WINDOW_NORMAL);
    
    // 生成图像1
    cv::Mat frame1 = sim.generate_frame(
        cv::Scalar(0, 0, 255),  // 中心红色
        colors1,                // 外围颜色
        "pentagon",             // 布局
        cv::Point2f(400, 300),  // 中心位置
        0.0f,                   // 无旋转
        2                       // 目标索引
    );
    
    // 生成图像2
    cv::Mat frame2 = sim.generate_frame(
        cv::Scalar(0, 0, 255),
        colors1,
        "pentagon",
        sim.get_random_position(),  // 随机位置
        0.0f,                       // 无旋转
        2
    );
    
    // 生成图像3
    cv::Mat frame3 = sim.generate_frame(
        cv::Scalar(0, 0, 255),
        colors1,
        "pentagon",
        sim.get_random_position(),                   // 随机位置
        static_cast<float>(rand()) / RAND_MAX * 2 * M_PI, // 随机旋转
        2
    );
    
    // 生成图像4
    std::vector<cv::Scalar> random_colors = sim.get_random_colors(5);
    cv::Mat frame4 = sim.generate_frame(
        cv::Scalar(0, 0, 255),
        random_colors,          // 随机颜色
        "pentagon",
        cv::Point2f(400, 300),
        0.0f,
        2
    );
    
    // 生成图像5
    cv::Mat frame5 = sim.generate_random_all_frame(2);
    
    // 在图像上添加文字标签
    cv::putText(frame1, "Fixed Position & Colors", cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);
    cv::putText(frame2, "Random Position", cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);
    cv::putText(frame3, "Random Position + Rotation", cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);
    cv::putText(frame4, "Random Colors", cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);
    cv::putText(frame5, "Fully Random", cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);
    

    // 或者逐个显示
   cv::imshow("Fixed Position", frame1);
   cv::imshow("Random Position", frame2);
   cv::imshow("Random Pose", frame3);
   cv::imshow("Random Colors", frame4);
   cv::imshow("Fully Random", frame5);
    cv::waitKey(0);
    
    return 0;
}