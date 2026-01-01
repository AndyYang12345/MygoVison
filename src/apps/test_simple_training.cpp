// test_simple_training.cpp - 简化测试程序
#include "TrainingFrameGenerator.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    std::cout << "简化训练帧生成器测试" << std::endl;
    std::cout << "====================" << std::endl;
    
    // 创建训练帧生成器
    TrainingFrameGenerator generator(800, 600, 30.0f);
    
    // 测试模式1：五角星旋转
    std::cout << "\n测试模式1：五角星旋转（角速度1.5 rad/s）" << std::endl;
    generator.set_training_mode(TrainingFrameGenerator::MODE_PENTAGON_ROTATION, 1.5f);
    
    cv::namedWindow("五角星旋转模式", cv::WINDOW_AUTOSIZE);
    for (int i = 0; i < 100; i++) {
        auto frame_data = generator.get_next_frame();
        
        // 在图像上显示信息
        std::string info = "模式: 五角星旋转 | 时间: " + 
                          std::to_string(frame_data.timestamp).substr(0, 4) + "s";
        cv::putText(frame_data.frame, info, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
        
        // 标记目标位置
        cv::circle(frame_data.frame, frame_data.target_position, 8, 
                  cv::Scalar(0, 255, 255), 2);
        
        cv::imshow("五角星旋转模式", frame_data.frame);
        if (cv::waitKey(30) == 27) break;
    }
    
    // 测试模式2：线性移动
    std::cout << "\n测试模式2：线性移动（速度50, 30 px/s）" << std::endl;
    generator.set_training_mode(TrainingFrameGenerator::MODE_LINEAR_MOVEMENT, 50.0f, 30.0f);
    
    cv::namedWindow("线性移动模式", cv::WINDOW_AUTOSIZE);
    for (int i = 0; i < 100; i++) {
        auto frame_data = generator.get_next_frame();
        
        std::string info = "模式: 线性移动 | 时间: " + 
                          std::to_string(frame_data.timestamp).substr(0, 4) + "s";
        cv::putText(frame_data.frame, info, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
        
        cv::circle(frame_data.frame, frame_data.target_position, 8, 
                  cv::Scalar(0, 255, 255), 2);
        
        cv::imshow("线性移动模式", frame_data.frame);
        if (cv::waitKey(30) == 27) break;
    }
    
    // 测试模式3：随机出现
    std::cout << "\n测试模式3：随机出现（间隔2秒）" << std::endl;
    generator.set_training_mode(TrainingFrameGenerator::MODE_RANDOM_APPEARANCE, 2.0f);
    
    cv::namedWindow("随机出现模式", cv::WINDOW_AUTOSIZE);
    for (int i = 0; i < 150; i++) {
        auto frame_data = generator.get_next_frame();
        
        std::string info = "模式: 随机出现 | 时间: " + 
                          std::to_string(frame_data.timestamp).substr(0, 4) + "s";
        cv::putText(frame_data.frame, info, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
        
        cv::circle(frame_data.frame, frame_data.target_position, 8, 
                  cv::Scalar(0, 255, 255), 2);
        
        cv::imshow("随机出现模式", frame_data.frame);
        if (cv::waitKey(30) == 27) break;
    }
    
    cv::destroyAllWindows();
    std::cout << "\n测试完成!" << std::endl;
    
    return 0;
}