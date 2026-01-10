#include "TargetSim/TargetSim.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

// 颜色定义
const cv::Scalar RED(0, 0, 255);
const cv::Scalar GREEN(0, 255, 0);
const cv::Scalar BLUE(255, 0, 0);
const cv::Scalar YELLOW(0, 255, 255);
const cv::Scalar CYAN(255, 255, 0);
const cv::Scalar MAGENTA(255, 0, 255);
const cv::Scalar BLACK(0, 0, 0);
const cv::Scalar WHITE(255, 255, 255);
const cv::Scalar GRAY(200, 200, 200);

/**
 * @brief 绘制详细的靶子分析图
 */
cv::Mat draw_detailed_analysis(const cv::Mat& frame, 
                              const std::vector<cv::Point2f>& centroids,
                              const cv::Point2f& target_position,
                              const cv::Point2f& detected_target) {
    cv::Mat result = frame.clone();
    
    // 1. 标记所有色块
    for (size_t i = 0; i < centroids.size(); i++) {
        std::string label;
        cv::Scalar color;
        int radius = 35;
        
        if (i == 0) {
            label = "center";
            color = YELLOW;
            radius = 40;
        } else {
            label = std::to_string(i-1);
            // 检查是否是目标色块（与目标位置匹配）
            if (std::abs(centroids[i].x - target_position.x) < 1.0f &&
                std::abs(centroids[i].y - target_position.y) < 1.0f) {
                color = GREEN;  // 目标色块：绿色
                radius = 45;
            } else {
                color = GRAY;   // 其他色块：灰色
            }
        }
        
        // 绘制色块外框
        cv::circle(result, centroids[i], radius, color, 3);
        
        // 添加标签
        cv::putText(result, label, 
                   cv::Point(centroids[i].x - 10, centroids[i].y + 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
        
        // 显示坐标
        std::string coord = "(" + std::to_string((int)centroids[i].x) + 
                           "," + std::to_string((int)centroids[i].y) + ")";
        cv::putText(result, coord, 
                   cv::Point(centroids[i].x - 25, centroids[i].y + 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, BLACK, 1);
    }
    
    // 2. 绘制从中心到目标的连接线
    if (target_position.x >= 0 && target_position.y >= 0) {
        cv::line(result, centroids[0], target_position, RED, 3);
        
        // 在线段中间添加"目标"标签
        cv::Point2f mid_point(
            (centroids[0].x + target_position.x) / 2,
            (centroids[0].y + target_position.y) / 2
        );
        cv::putText(result, "target match line", 
                   cv::Point(mid_point.x - 40, mid_point.y - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, RED, 2);
    }
    
    // 3. 如果提供了检测到的目标位置，用红色"X"标记
    if (detected_target.x >= 0) {
        int cross_size = 20;
        cv::line(result, 
                cv::Point(detected_target.x - cross_size, detected_target.y - cross_size),
                cv::Point(detected_target.x + cross_size, detected_target.y + cross_size),
                cv::Scalar(255, 0, 255), 4);  // 紫色X
        
        cv::line(result, 
                cv::Point(detected_target.x + cross_size, detected_target.y - cross_size),
                cv::Point(detected_target.x - cross_size, detected_target.y + cross_size),
                cv::Scalar(255, 0, 255), 4);
        
        cv::putText(result, "detected target", 
                   cv::Point(detected_target.x + 25, detected_target.y - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);
    }
    
    return result;
}

/**
 * @brief 测试单一目标位置
 */
bool test_single_target_position(TargetSim& sim, 
                                const cv::Point2f& center, float rotation_angle,
                                int test_num) {
    std::cout << "\n=== 测试 " << test_num 
              << " (旋转 " << (rotation_angle * 180 / M_PI) << "°) ===" << std::endl;
    
    // 生成靶子图像并获取目标位置
    cv::Point2f target_position;
    cv::Mat frame = sim.generate_pentagon_frame(center, rotation_angle, &target_position);
    
    // 获取理论上的色块中心
    std::vector<cv::Point2f> centroids = sim.get_blob_centroids();
    cv::Point2f last_target_position = sim.get_last_target_position();
    
    std::cout << "返回的目标位置: (" << std::fixed << std::setprecision(1) 
              << target_position.x << ", " << target_position.y << ")" << std::endl;
    std::cout << "存储的目标位置: (" << last_target_position.x << ", " 
              << last_target_position.y << ")" << std::endl;
    
    // 验证目标位置是否正确（在色块中心列表中）
    bool position_correct = false;
    int matched_index = -1;
    
    for (size_t i = 1; i < centroids.size(); i++) {
        float distance = cv::norm(target_position - centroids[i]);
        if (distance < 1.0f) {  // 允许1像素误差
            position_correct = true;
            matched_index = i - 1;
            std::cout << "匹配到外围色块 " << matched_index << std::endl;
            break;
        }
    }
    
    if (!position_correct) {
        std::cout << "警告：目标位置未匹配到任何外围色块！" << std::endl;
        std::cout << "所有外围色块位置：" << std::endl;
        for (size_t i = 1; i < centroids.size(); i++) {
            std::cout << "  色块 " << (i-1) << ": (" << centroids[i].x 
                      << ", " << centroids[i].y << ")" << std::endl;
        }
    }
    
    // 验证存储的位置与返回的位置是否一致
    bool storage_correct = (cv::norm(target_position - last_target_position) < 1.0f);
    
    // 绘制详细分析图
    cv::Mat analysis_frame = draw_detailed_analysis(frame, centroids, target_position, target_position);
    
    // 添加测试信息
    std::string status = (position_correct && storage_correct) ? "✅ 通过" : "❌ 失败";
    cv::putText(analysis_frame, "test" + std::to_string(test_num) + ": " + status,
               cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, 
               (position_correct && storage_correct) ? GREEN : RED, 2);
    
    cv::putText(analysis_frame, "target position: (" + 
               std::to_string((int)target_position.x) + ", " + 
               std::to_string((int)target_position.y) + ")",
               cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.7, BLACK, 2);
    
    if (matched_index >= 0) {
        cv::putText(analysis_frame, "matched blob: " + std::to_string(matched_index),
                   cv::Point(10, 100), cv::FONT_HERSHEY_SIMPLEX, 0.7, BLUE, 1);
    }
    
    cv::putText(analysis_frame, "rotation angle: " + 
               std::to_string((int)(rotation_angle * 180 / M_PI)) + "°",
               cv::Point(10, 130), cv::FONT_HERSHEY_SIMPLEX, 0.7, BLACK, 1);
    
    // 显示结果
    std::string window_name = "test " + std::to_string(test_num);
    cv::imshow(window_name, analysis_frame);
    
    std::cout << "位置验证: " << (position_correct ? "✅" : "❌") << std::endl;
    std::cout << "存储验证: " << (storage_correct ? "✅" : "❌") << std::endl;
    std::cout << "结果: " << status << std::endl;
    
    return position_correct && storage_correct;
}

/**
 * @brief 测试所有旋转角度
 */
void test_all_rotations(TargetSim& sim, const cv::Point2f& center) {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "测试不同旋转角度 (位置: " << center.x << ", " << center.y << ")" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    int passed_tests = 0;
    int total_tests = 8;
    
    // 测试不同旋转角度
    float angles[] = {0.0f, M_PI/6, M_PI/4, M_PI/3, M_PI/2, 
                      2*M_PI/3, M_PI, 3*M_PI/2};
    
    for (int i = 0; i < total_tests; i++) {
        bool passed = test_single_target_position(sim, center, angles[i], i + 1);
        if (passed) passed_tests++;
        
        cv::waitKey(1000);  // 显示1秒
        if (i < total_tests - 1) {
            // 销毁当前窗口，准备下一个
            cv::destroyAllWindows();
        }
    }
    
    std::cout << "\n" << std::string(60, '-') << std::endl;
    std::cout << "汇总: " << passed_tests << "/" << total_tests << " 测试通过" << std::endl;
    std::cout << std::string(60, '-') << std::endl;
    
    cv::waitKey(2000);  // 最后显示2秒
}

/**
 * @brief 测试随机位置
 */
void test_random_positions(TargetSim& sim) {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "测试随机位置的目标识别" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    srand(time(nullptr));
    
    for (int i = 0; i < 3; i++) {
        // 随机位置
        cv::Point2f random_center = sim.get_random_position(200.0f);
        
        // 随机旋转角度
        float random_angle = (rand() % 360) * M_PI / 180.0f;
        
        std::cout << "\n随机测试 " << (i+1) << ":" << std::endl;
        std::cout << "位置: (" << random_center.x << ", " << random_center.y << ")" << std::endl;
        std::cout << "旋转角度: " << (random_angle * 180 / M_PI) << "°" << std::endl;
        
        cv::Point2f target_position;
        cv::Mat frame = sim.generate_pentagon_frame(random_center, random_angle, &target_position);
        std::vector<cv::Point2f> centroids = sim.get_blob_centroids();
        
        std::cout << "目标位置: (" << target_position.x << ", " << target_position.y << ")" << std::endl;
        
        cv::Mat analysis = draw_detailed_analysis(frame, centroids, target_position, target_position);
        
        cv::putText(analysis, "random test " + std::to_string(i+1),
                   cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, MAGENTA, 2);
        
        cv::imshow("random test " + std::to_string(i+1), analysis);
        cv::waitKey(2000);
        cv::destroyAllWindows();
    }
}

/**
 * @brief 交互式目标测试
 */
void interactive_target_test(TargetSim& sim) {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "交互式目标位置测试" << std::endl;
    std::cout << "按键说明:" << std::endl;
    std::cout << "  + -: 增加/减少旋转角度" << std::endl;
    std::cout << "  r  : 重置旋转角度和颜色" << std::endl;
    std::cout << "  n  : 新颜色组合" << std::endl;
    std::cout << "  c  : 随机改变位置" << std::endl;
    std::cout << "  ESC: 退出测试" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    cv::Point2f center(400, 300);
    float current_angle = 0.0f;
    float angle_step = M_PI / 18.0f;  // 10度
    
    while (true) {
        // 生成当前设置的靶子
        cv::Point2f target_position;
        cv::Mat frame = sim.generate_pentagon_frame(center, current_angle, &target_position);
        
        // 获取理论数据
        std::vector<cv::Point2f> centroids = sim.get_blob_centroids();
        // cv::Point2f last_target_position = sim.get_last_target_position(); // 移除未使用的变量
        
        // 绘制分析
        cv::Mat display = draw_detailed_analysis(frame, centroids, target_position, target_position);
        
        // 添加控制信息（重新绘制每次）
        cv::rectangle(display, cv::Point(5, 5), cv::Point(350, 160), cv::Scalar(255, 255, 255), -1);
        cv::rectangle(display, cv::Point(5, 5), cv::Point(350, 160), cv::Scalar(0, 0, 0), 1);
        
        std::string info = "target position: (" + 
                          std::to_string((int)target_position.x) + ", " + 
                          std::to_string((int)target_position.y) + ")";
        cv::putText(display, info, cv::Point(10, 40), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, GREEN, 2);
        
        std::string angle_info = "rotation angle: " +
                                std::to_string((int)(current_angle * 180 / M_PI)) + "°";
        cv::putText(display, angle_info, cv::Point(10, 70), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, BLUE, 2);
        
        std::string pos_info = "center position: (" + 
                              std::to_string((int)center.x) + ", " + 
                              std::to_string((int)center.y) + ")";
        cv::putText(display, pos_info, cv::Point(10, 100), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, BLACK, 1);
        
        // 添加目标颜色信息
        cv::Scalar target_color = sim.get_last_target_color();
        std::string color_info = "target color: BGR(" + 
                               std::to_string((int)target_color[0]) + "," +
                               std::to_string((int)target_color[1]) + "," +
                               std::to_string((int)target_color[2]) + ")";
        cv::putText(display, color_info, cv::Point(10, 130), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(128, 0, 128), 1);
        
        cv::imshow("交互式目标测试", display);
        
        // 处理按键
        int key = cv::waitKey(30);
        if (key == 27) {  // ESC
            break;
        } else if (key == '+') {
            current_angle += angle_step;
            if (current_angle > 2 * M_PI) current_angle -= 2 * M_PI;
            std::cout << "角度增加至: " << (current_angle * 180 / M_PI) << "°" << std::endl;
        } else if (key == '-') {
            current_angle -= angle_step;
            if (current_angle < 0) current_angle += 2 * M_PI;
            std::cout << "角度减少至: " << (current_angle * 180 / M_PI) << "°" << std::endl;
        } else if (key == 'r' || key == 'R') {
    current_angle = 0.0f;
    // 使用新的 regenerate_colors 方法
    sim.regenerate_colors();
    // 也可以选择随机位置
    center = sim.get_random_position(200.0f);
    std::cout << "重置：角度=0°，新位置: (" << center.x << ", " << center.y << ")" << std::endl;
    } else if (key == 'n' || key == 'N') {
        // 新颜色组合 - 使用新的 regenerate_colors 方法
        sim.regenerate_colors();
        // 生成一帧来应用新颜色
        sim.generate_pentagon_frame(center, current_angle, nullptr);
        std::cout << "新颜色组合已生成" << std::endl;
    } else if (key == 'c' || key == 'C') {
            center = sim.get_random_position(200.0f);
            std::cout << "新位置: (" << center.x << ", " << center.y << ")" << std::endl;
        }
    }
    
    cv::destroyAllWindows();
}

int main() {
    std::cout << "🎯 靶子目标色块位置验证测试程序" << std::endl;
    std::cout << "================================" << std::endl;
    
    // 创建靶子仿真器
    TargetSim sim(800, 600, cv::Scalar(240, 240, 240));
    
    // 测试1: 不同旋转角度
    test_all_rotations(sim, cv::Point2f(400, 300));
    
    // 测试2: 随机位置
    test_random_positions(sim);
    
    // 测试3: 交互式测试
    interactive_target_test(sim);
    
    std::cout << "\n✅ 所有测试完成!" << std::endl;
    std::cout << "程序结束" << std::endl;
    
    return 0;
}