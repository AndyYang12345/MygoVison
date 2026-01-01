// src/apps/test_target_verification.cpp
#include "targetSim/targetSim.hpp"
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
                              int target_idx,
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
            if (static_cast<int>(i-1) == target_idx) {
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
    if (target_idx >= 0 && centroids.size() > static_cast<size_t>(target_idx + 1)) {
        cv::Point2f target_pos = centroids[target_idx + 1];
        cv::line(result, centroids[0], target_pos, RED, 3);
        
        // 在线段中间添加"目标"标签
        cv::Point2f mid_point(
            (centroids[0].x + target_pos.x) / 2,
            (centroids[0].y + target_pos.y) / 2
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
 * @brief 测试单一目标索引
 */
bool test_single_target_index(TargetSim& sim, int target_idx, 
                             const cv::Point2f& center, float rotation_angle,
                             int test_num) {
    std::cout << "\n=== 测试 " << test_num << ": 目标索引 " << target_idx 
              << " (旋转 " << (rotation_angle * 180 / M_PI) << "°) ===" << std::endl;
    
    // 生成靶子图像
    cv::Mat frame = sim.generate_pentagon_frame(center, rotation_angle, target_idx);
    
    // 获取理论上的色块中心
    std::vector<cv::Point2f> centroids = sim.get_blob_centroids();
    
    // 获取理论上的目标位置
    cv::Point2f theoretical_target = sim.get_target_position();
    
    // 获取当前目标索引（从类内部）
    int actual_target_idx = sim.get_current_target_idx();
    
    std::cout << "设置的目标索引: " << target_idx << std::endl;
    std::cout << "内部存储的目标索引: " << actual_target_idx << std::endl;
    std::cout << "理论目标位置: (" << std::fixed << std::setprecision(1) 
              << theoretical_target.x << ", " << theoretical_target.y << ")" << std::endl;
    
    // 验证目标索引是否正确
    bool index_correct = (actual_target_idx == target_idx);
    
    // 验证目标位置是否正确（如果centroids足够）
    bool position_correct = false;
    if (centroids.size() > static_cast<size_t>(target_idx + 1)) {
        cv::Point2f expected_target = centroids[target_idx + 1];
        float distance = cv::norm(theoretical_target - expected_target);
        position_correct = (distance < 1.0f);  // 允许1像素误差
        
        std::cout << "期望的目标位置: (" << expected_target.x << ", " 
                  << expected_target.y << ")" << std::endl;
        std::cout << "位置误差: " << distance << " 像素" << std::endl;
    }
    
    // 绘制详细分析图
    cv::Mat analysis_frame = draw_detailed_analysis(frame, centroids, target_idx, 
                                                   theoretical_target);
    
    // 添加测试信息
    std::string status = (index_correct && position_correct) ? "✅ 通过" : "❌ 失败";
    cv::putText(analysis_frame, "test" + std::to_string(test_num) + ": " + status,
               cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, 
               (index_correct && position_correct) ? GREEN : RED, 2);
    
    cv::putText(analysis_frame, "target index: " + std::to_string(target_idx),
               cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.7, BLACK, 2);
    
    cv::putText(analysis_frame, "rotation angle: " + 
               std::to_string((int)(rotation_angle * 180 / M_PI)) + "°",
               cv::Point(10, 100), cv::FONT_HERSHEY_SIMPLEX, 0.7, BLACK, 1);
    
    // 显示结果
    std::string window_name = "test " + std::to_string(test_num) + 
                             " - target index " + std::to_string(target_idx);
    cv::imshow(window_name, analysis_frame);
    
    std::cout << "结果: " << status << std::endl;
    
    return index_correct && position_correct;
}

/**
 * @brief 测试所有目标索引（0-4）
 */
void test_all_target_indices(TargetSim& sim, const cv::Point2f& center) {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "测试所有目标索引 (位置: " << center.x << ", " << center.y << ")" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    int passed_tests = 0;
    int total_tests = 5;
    
    // 测试每个目标索引
    for (int target_idx = 0; target_idx < total_tests; target_idx++) {
        bool passed = test_single_target_index(sim, target_idx, center, 0.0f, target_idx + 1);
        if (passed) passed_tests++;
        
        cv::waitKey(1000);  // 显示1秒
        if (target_idx < total_tests - 1) {
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
 * @brief 测试旋转对目标位置的影响
 */
void test_target_with_rotation(TargetSim& sim) {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "测试旋转对目标位置的影响" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    cv::Point2f center(400, 300);
    int target_idx = 2;  // 固定目标索引
    
    // 测试不同旋转角度
    float angles[] = {0.0f, M_PI/6, M_PI/4, M_PI/2, M_PI, 3*M_PI/2};
    std::string angle_names[] = {"0°", "30°", "45°", "90°", "180°", "270°"};
    
    for (int i = 0; i < 6; i++) {
        std::cout << "\n旋转角度: " << angle_names[i] << std::endl;
        
        cv::Mat frame = sim.generate_pentagon_frame(center, angles[i], target_idx);
        std::vector<cv::Point2f> centroids = sim.get_blob_centroids();
        cv::Point2f target_pos = sim.get_target_position();
        
        if (centroids.size() > static_cast<size_t>(target_idx + 1)) {
            cv::Point2f expected_pos = centroids[target_idx + 1];
            float error = cv::norm(target_pos - expected_pos);
            
            std::cout << "目标索引: " << target_idx << std::endl;
            std::cout << "理论位置: (" << target_pos.x << ", " << target_pos.y << ")" << std::endl;
            std::cout << "实际位置: (" << expected_pos.x << ", " << expected_pos.y << ")" << std::endl;
            std::cout << "位置误差: " << std::fixed << std::setprecision(2) << error << " 像素" << std::endl;
            
            if (error < 1.0f) {
                std::cout << "✅ 位置准确" << std::endl;
            } else {
                std::cout << "❌ 位置偏移过大" << std::endl;
            }
        }
        
        cv::Mat analysis = draw_detailed_analysis(frame, centroids, target_idx, target_pos);
        cv::putText(analysis, "旋转角度: " + angle_names[i],
                   cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, BLUE, 2);
        
        cv::imshow("旋转测试: " + angle_names[i], analysis);
        cv::waitKey(1500);
        cv::destroyAllWindows();
    }
}

/**
 * @brief 测试随机位置的目标识别
 */
void test_random_positions(TargetSim& sim) {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "测试随机位置的目标识别" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    srand(time(nullptr));
    
    for (int i = 0; i < 3; i++) {
        // 随机位置
        cv::Point2f random_center = sim.get_random_position(200.0f);
        
        // 随机目标索引
        int random_target_idx = rand() % 5;
        
        // 随机旋转角度
        float random_angle = (rand() % 360) * M_PI / 180.0f;
        
        std::cout << "\n随机测试 " << (i+1) << ":" << std::endl;
        std::cout << "位置: (" << random_center.x << ", " << random_center.y << ")" << std::endl;
        std::cout << "目标索引: " << random_target_idx << std::endl;
        std::cout << "旋转角度: " << (random_angle * 180 / M_PI) << "°" << std::endl;
        
        cv::Mat frame = sim.generate_pentagon_frame(random_center, random_angle, random_target_idx);
        std::vector<cv::Point2f> centroids = sim.get_blob_centroids();
        cv::Point2f target_pos = sim.get_target_position();
        
        cv::Mat analysis = draw_detailed_analysis(frame, centroids, random_target_idx, target_pos);
        
        cv::putText(analysis, "random test " + std::to_string(i+1),
                   cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, MAGENTA, 2);
        
        cv::imshow("random test " + std::to_string(i+1), analysis);
        cv::waitKey(2000);
        cv::destroyAllWindows();
    }
}

/**
 * @brief 交互式目标索引测试
 */
void interactive_target_test(TargetSim& sim) {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "交互式目标索引测试" << std::endl;
    std::cout << "按键说明:" << std::endl;
    std::cout << "  0-4: 切换目标索引" << std::endl;
    std::cout << "  + -: 增加/减少旋转角度" << std::endl;
    std::cout << "  r  : 重置旋转角度" << std::endl;
    std::cout << "  c  : 随机改变位置" << std::endl;
    std::cout << "  ESC: 退出测试" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    cv::Point2f center(400, 300);
    int current_target_idx = 0;
    float current_angle = 0.0f;
    float angle_step = M_PI / 18.0f;  // 10度
    
    while (true) {
        // 生成当前设置的靶子
        cv::Mat frame = sim.generate_pentagon_frame(center, current_angle, current_target_idx);
        
        // 获取理论数据
        std::vector<cv::Point2f> centroids = sim.get_blob_centroids();
        cv::Point2f target_pos = sim.get_target_position();
        int actual_target_idx = sim.get_current_target_idx();
        
        // 绘制分析
        cv::Mat display = draw_detailed_analysis(frame, centroids, current_target_idx, target_pos);
        
        // 添加控制信息
        std::string info = "target index: " + std::to_string(current_target_idx) + 
                          " (actual: " + std::to_string(actual_target_idx) + ")";
        cv::putText(display, info, cv::Point(10, 40), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, 
                   (current_target_idx == actual_target_idx) ? GREEN : RED, 2);
        
        std::string angle_info = "rotation angle: " +
                                std::to_string((int)(current_angle * 180 / M_PI)) + "°";
        cv::putText(display, angle_info, cv::Point(10, 70), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, BLUE, 2);
        
        std::string pos_info = "position: (" + 
                              std::to_string((int)center.x) + ", " + 
                              std::to_string((int)center.y) + ")";
        cv::putText(display, pos_info, cv::Point(10, 100), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, BLACK, 1);
        
        cv::imshow("交互式目标测试", display);
        
        // 处理按键
        int key = cv::waitKey(30);
        if (key == 27) {  // ESC
            break;
        } else if (key >= '0' && key <= '4') {
            current_target_idx = key - '0';
            std::cout << "切换到目标索引: " << current_target_idx << std::endl;
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
            std::cout << "角度重置为0°" << std::endl;
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
    
    // 测试1: 所有目标索引
    test_all_target_indices(sim, cv::Point2f(400, 300));
    
    // 测试2: 旋转对目标位置的影响
    test_target_with_rotation(sim);
    
    // 测试3: 随机位置
    test_random_positions(sim);
    
    // 测试4: 交互式测试
    interactive_target_test(sim);
    
    std::cout << "\n✅ 所有测试完成!" << std::endl;
    std::cout << "程序结束" << std::endl;
    
    return 0;
}