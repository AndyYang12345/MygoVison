// test_final_working.cpp
#include "TargetTracking/TargetTracker.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
    // 加载图像
    cv::Mat img = cv::imread("test_input.jpg");
    if (img.empty()) {
        std::cerr << "❌ Failed to load image!" << std::endl;
        return -1;
    }
    
    std::cout << "=== TARGET TRACKER - FINAL WORKING VERSION ===" << std::endl;
    std::cout << "Image: " << img.cols << "x" << img.rows << std::endl;
    
    // 创建追踪器并使用最佳配置
    TargetTracker tracker;
    TrackerConfig config = tracker.get_config();
    
    // 最佳参数配置（经过测试的）
    config.saturation_threshold = 40;
    config.value_threshold = 90;
    config.min_blob_area = 1200;
    config.max_blob_area = 9000;
    config.min_circularity = 0.3f;
    config.min_distance_to_center = 80.0f;
    config.max_distance_to_center = 200.0f;
    config.hue_similarity_threshold = 18.0f;
    config.bgr_distance_threshold = 60.0f;
    config.dark_brightness_threshold = 120;
    
    // 启用调试输出，但禁用图形显示（避免hconcat错误）
    config.print_debug_info = true;
    config.show_debug_windows = false;  // 改为false避免显示错误
    
    tracker.set_config(config);
    
    std::cout << "\n=== 运行目标追踪 ===" << std::endl;
    
    // 运行追踪器
    auto result = tracker.process_frame(img);
    
    if (result.found) {
        std::cout << "\n🎯 TARGET TRACKING SUCCESS!" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "Board Center:     (" << result.board_center.x 
                  << ", " << result.board_center.y << ")" << std::endl;
        std::cout << "Target Center:    (" << result.target_center.x 
                  << ", " << result.target_center.y << ")" << std::endl;
        std::cout << "Distance:         " << result.distance << " pixels" << std::endl;
        std::cout << "Angle:            " << result.angle << " degrees" << std::endl;
        std::cout << "========================================\n" << std::endl;
        
        // 创建可视化结果（安全的实现）
        cv::Mat visual_result = img.clone();
        
        // 绘制中心色块
        cv::circle(visual_result, result.board_center, 12, cv::Scalar(0, 255, 255), 3);
        cv::putText(visual_result, "CENTER", 
                   cv::Point(result.board_center.x + 15, result.board_center.y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        
        // 绘制目标色块
        cv::circle(visual_result, result.target_center, 10, cv::Scalar(0, 0, 255), 3);
        cv::putText(visual_result, "TARGET", 
                   cv::Point(result.target_center.x + 15, result.target_center.y),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
        
        // 绘制连接线
        cv::line(visual_result, result.board_center, result.target_center,
                cv::Scalar(0, 255, 0), 2);
        
        // 添加距离和角度信息
        std::string dist_text = cv::format("Distance: %.1f px", result.distance);
        std::string angle_text = cv::format("Angle: %.1f deg", result.angle);
        
        cv::putText(visual_result, dist_text, cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        cv::putText(visual_result, angle_text, cv::Point(10, 60),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        
        // 保存结果
        cv::imwrite("tracking_success.jpg", visual_result);
        std::cout << "✅ 结果保存到: tracking_success.jpg" << std::endl;
        
        // 安全地显示图像
        cv::imshow("Tracking Result", visual_result);
        std::cout << "\n显示追踪结果，按任意键继续..." << std::endl;
        cv::waitKey(0);
        
    } else {
        std::cout << "\n❌ TARGET NOT FOUND" << std::endl;
    }
    
    // ========== 批量测试验证鲁棒性 ==========
    
    std::cout << "\n=== 鲁棒性测试 ===" << std::endl;
    
    // 测试不同尺度的图像
    std::vector<float> scales = {0.5f, 0.75f, 1.0f, 1.25f, 1.5f};
    int scale_success = 0;
    
    for (float scale : scales) {
        cv::Mat scaled_img;
        cv::resize(img, scaled_img, cv::Size(), scale, scale);
        
        auto scale_result = tracker.process_frame(scaled_img);
        
        std::cout << "缩放 " << (scale*100) << "%: ";
        if (scale_result.found) {
            scale_success++;
            std::cout << "✅" << std::endl;
        } else {
            std::cout << "❌" << std::endl;
        }
    }
    
    std::cout << "缩放测试成功率: " << scale_success << "/" << scales.size() << std::endl;
    
    // ========== 生成配置头文件 ==========
    
    std::cout << "\n=== 最终配置代码 ===" << std::endl;
    std::cout << "\n将以下配置添加到 TargetTracker.hpp 的构造函数中:\n" << std::endl;
    
    std::cout << "// 在 TargetTracker 构造函数中:" << std::endl;
    std::cout << "TargetTracker::TargetTracker() {" << std::endl;
    std::cout << "    // 最佳参数配置" << std::endl;
    std::cout << "    config_.saturation_threshold = 40;" << std::endl;
    std::cout << "    config_.value_threshold = 90;" << std::endl;
    std::cout << "    config_.min_blob_area = 1200;" << std::endl;
    std::cout << "    config_.max_blob_area = 9000;" << std::endl;
    std::cout << "    config_.min_circularity = 0.3f;" << std::endl;
    std::cout << "    config_.min_distance_to_center = 80.0f;" << std::endl;
    std::cout << "    config_.max_distance_to_center = 200.0f;" << std::endl;
    std::cout << "    config_.hue_similarity_threshold = 18.0f;" << std::endl;
    std::cout << "    config_.bgr_distance_threshold = 60.0f;" << std::endl;
    std::cout << "    config_.dark_brightness_threshold = 120;" << std::endl;
    std::cout << "    " << std::endl;
    std::cout << "    // 初始化其他成员变量" << std::endl;
    std::cout << "    total_frames_ = 0;" << std::endl;
    std::cout << "    successful_detections_ = 0;" << std::endl;
    std::cout << "    debug_mode_ = false;" << std::endl;
    std::cout << "}" << std::endl;
    
    // ========== 修复TargetTracker.cpp中的显示问题 ==========
    
    std::cout << "\n=== 修复显示问题 ===" << std::endl;
    std::cout << "\n如果还需要显示功能，请修改 TargetTracker.cpp 中的显示代码:" << std::endl;
    std::cout << "\n在 process_frame 函数的显示部分，将:" << std::endl;
    std::cout << "\n// 原来的hconcat可能有问题:" << std::endl;
    std::cout << "vector<Mat> images = {debug_frame, mask_display};" << std::endl;
    std::cout << "hconcat(images, combined);" << std::endl;
    std::cout << "\n改为更安全的实现:" << std::endl;
    std::cout << "// 确保两个图像尺寸相同" << std::endl;
    std::cout << "if (debug_frame.size() == mask_display.size()) {" << std::endl;
    std::cout << "    vector<Mat> images = {debug_frame, mask_display};" << std::endl;
    std::cout << "    hconcat(images, combined);" << std::endl;
    std::cout << "} else {" << std::endl;
    std::cout << "    // 调整mask_display尺寸以匹配debug_frame" << std::endl;
    std::cout << "    cv::resize(mask_display, mask_display, debug_frame.size());" << std::endl;
    std::cout << "    vector<Mat> images = {debug_frame, mask_display};" << std::endl;
    std::cout << "    hconcat(images, combined);" << std::endl;
    std::cout << "}" << std::endl;
    
    std::cout << "\nPress Enter to exit..." << std::endl;
    std::cin.get();
    
    return 0;
}