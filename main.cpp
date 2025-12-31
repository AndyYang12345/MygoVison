#include "targetSim.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    // 1. 初始化仿真器
    TargetSim sim(800, 600, cv::Scalar(220, 220, 220)); // 浅灰背景
    cv::Scalar red(0, 0, 255);
    
    // 5个不同颜色的外围色块，方便观察旋转
    std::vector<cv::Scalar> colors = {
        cv::Scalar(0, 255, 0),     // 绿
        cv::Scalar(255, 0, 0),     // 蓝
        cv::Scalar(255, 255, 0),   // 青
        cv::Scalar(255, 0, 255),   // 紫
        cv::Scalar(0, 255, 255)    // 黄
    };
    
    // 2. 随机选择一个靶子中心位置（只选一次）
    float margin = 120.0f;
    cv::Point2f random_center(
        margin + static_cast<float>(rand()) / RAND_MAX * (800 - 2*margin),
        margin + static_cast<float>(rand()) / RAND_MAX * (600 - 2*margin)
    );
    
    std::cout << "靶子中心位置: (" << random_center.x << ", " << random_center.y << ")" << std::endl;
    std::cout << "按ESC键退出旋转..." << std::endl;
    
    // 3. 旋转参数
    float current_angle = 0.0f;
    float rotation_speed = 0.08f;  // 弧度/帧，调节旋转速度
    int target_idx = 2;            // 第3个外围色块与中心同色（红色）
    
    // 4. 主循环：在固定位置持续旋转
    while (true) {
        // 生成当前旋转角度的图像
        cv::Mat frame = sim.generate_frame(
            red,                    // 中心颜色
            colors,                 // 外围颜色
            "pentagon",             // 布局模式
            random_center,          // 固定随机位置
            current_angle,          // 当前旋转角度
            target_idx              // 目标色块索引
        );
        
        // 在图像上添加信息显示
        std::string angle_text = "角度: " + 
                                std::to_string(static_cast<int>(current_angle * 180 / CV_PI)) + 
                                "°";
        cv::putText(frame, angle_text, cv::Point(10, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);
        
        cv::putText(frame, "位置: (" + 
                          std::to_string(static_cast<int>(random_center.x)) + ", " + 
                          std::to_string(static_cast<int>(random_center.y)) + ")", 
                    cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
        
        // 绘制一个从中心到某个外围色块的指示线，更直观显示旋转
        // 获取所有色块的中心坐标（通过你的仿真器）
    std::vector<cv::Point2f> centers = sim.get_blob_centroids();

    // 确保有足够的色块中心
    if (centers.size() >= 6) {  // 中心1个 + 外围5个 = 6个
        // 中心色块坐标：centers[0]
        // 目标色块（第target_idx个外围色块）坐标：centers[target_idx + 1]
        cv::Point2f center_blob = centers[0];
        cv::Point2f target_blob = centers[target_idx + 1];
        
        // 绘制连接线（红色，加粗）
        cv::line(frame, center_blob, target_blob, 
                cv::Scalar(0, 0, 255),  // 红色线条
                3);                      // 线宽3像素
        
        // 可选：在目标色块上添加标记
        cv::circle(frame, target_blob, 8, cv::Scalar(0, 255, 255), 2); // 黄色圆圈标记
        cv::putText(frame, "TARGET", 
                    cv::Point(target_blob.x + 15, target_blob.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
}
        
        // 显示图像
        cv::imshow("随机位置自转靶子", frame);
        
        // 更新旋转角度
        current_angle += rotation_speed;
        if (current_angle > 2 * CV_PI) {
            current_angle -= 2 * CV_PI;
        }
        
        // 控制帧率并检测退出
        int key = cv::waitKey(30); // 约33fps
        if (key == 27) {           // ESC键退出
            break;
        } else if (key == ' ') {   // 空格键暂停/继续
            cv::waitKey(0);
        } else if (key == 'r') {   // R键重置角度
            current_angle = 0.0f;
        } else if (key == '+') {   // +键加速
            rotation_speed += 0.01f;
            std::cout << "旋转速度: " << rotation_speed << " 弧度/帧" << std::endl;
        } else if (key == '-') {   // -键减速
            rotation_speed = std::max(0.01f, rotation_speed - 0.01f);
            std::cout << "旋转速度: " << rotation_speed << " 弧度/帧" << std::endl;
        }
    }
    
    cv::destroyAllWindows();
    std::cout << "程序结束" << std::endl;
    return 0;
}