#include "TargetTracking/TargetTracker.hpp"
#include "TargetSim/TrainingFrameGenerator.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <functional>
#include <vector>
#include <random>
#include <cmath>
#include <fstream>  // 添加这个头文件
#include <sstream>  // 添加这个头文件

// ==================== 测试函数声明 ====================
void test_pentagon_rotation(TrainingFrameGenerator& generator);
void test_circular_motion(TrainingFrameGenerator& generator);
void test_spiral_motion(TrainingFrameGenerator& generator);
void test_sine_wave_motion(TrainingFrameGenerator& generator);
void test_linear_movement_with_bounce(TrainingFrameGenerator& generator);
void test_random_appearance(TrainingFrameGenerator& generator);
void test_lissajous_motion(TrainingFrameGenerator& generator);
void test_pic();
void auto_adjust_parameters(cv::Mat& test_image);
void display_info(cv::Mat& image, const std::string& title, 
                  const cv::Point2f& position, const cv::Point2f& velocity,
                  float timestamp) ;
void draw_trajectory(cv::Mat& image, const std::vector<cv::Point2f>& trajectory, 
                     const cv::Scalar& color) ;
// 先声明辅助函数，避免需要TargetColorInfo的定义
void record_failure_info_with_colors(TargetSim& target_sim, int target_id, float timestamp,
                                   float accuracy, float test_time, int total_frames, int success_frames,
                                   std::ofstream& failure_log, std::ofstream& color_log,
                                   std::ofstream& detailed_log);

void generate_color_analysis_report(const std::vector<struct TargetColorInfo>& problematic_infos);


struct TargetColorInfo {
    int target_id;
    std::string timestamp;
    float accuracy;
    int test_frames;
    std::string error_frame_file;
    
    // 添加缺少的成员变量
    cv::Scalar center_color_bgr;
    cv::Scalar target_color_bgr;
    std::vector<cv::Scalar> surround_colors_bgr;
    int target_index;  // 注意：这里要和结构体中其他成员区分开
    
    TargetColorInfo() : target_id(0), accuracy(0.0f), test_frames(0), target_index(-1) {}
};


// ==================== 主函数 ====================
int main() {
    std::cout << "Training Frame Generator Test Suite" << std::endl;
    std::cout << "===================================" << std::endl;
    
    // 创建 TrainingFrameGenerator
    TrainingFrameGenerator generator(450, 450, 80.0f);
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Testing simulator..." << std::endl;

    // 运行测试
    test_pentagon_rotation(generator);
    
    std::cout << "\n=== All tests completed! ===" << std::endl;
    
    return 0;
}


// ==================== 测试函数实现 ====================
/**
 * @brief 带颜色信息的故障记录函数（增强版，记录所有色块信息）
 */
void record_failure_info_with_colors(TargetSim& target_sim, int target_id, float timestamp,
                                   float accuracy, float test_time, int total_frames, int success_frames,
                                   std::ofstream& failure_log, std::ofstream& color_log,
                                   std::ofstream& detailed_log) {
    
    // 获取颜色信息
    cv::Scalar center_color = target_sim.get_center_color();
    cv::Scalar target_color = target_sim.get_target_color();
    int target_index = target_sim.get_target_index();
    std::vector<cv::Scalar> surround_colors = target_sim.get_surround_colors();
    
    // 获取当前时间
    time_t now = time(nullptr);
    std::string time_str = std::ctime(&now);
    time_str = time_str.substr(0, time_str.length() - 1);
    
    // 1. 记录到failure_log.txt
    failure_log << "\n=== 故障记录 (带颜色信息) ===" << std::endl;
    failure_log << "记录时间: " << time_str << std::endl;
    failure_log << "靶子ID: " << target_id << std::endl;
    failure_log << "时间戳: " << timestamp << "s" << std::endl;
    failure_log << "准确率: " << accuracy << "%" << std::endl;
    failure_log << "测试时间: " << test_time << "s" << std::endl;
    failure_log << "总帧数: " << total_frames << std::endl;
    failure_log << "成功帧数: " << success_frames << std::endl;
    failure_log << "目标索引: " << target_index << std::endl;
    
    // 2. 详细颜色信息记录到color_log.txt
    color_log << "\n=== 靶子ID: " << target_id << " ===" << std::endl;
    color_log << "时间戳: " << timestamp << "s" << std::endl;
    color_log << "准确率: " << accuracy << "%" << std::endl;
    color_log << "测试时间: " << test_time << "秒" << std::endl;
    color_log << "总帧数/成功帧数: " << total_frames << "/" << success_frames << std::endl;
    color_log << "目标索引: " << target_index << std::endl;
    
    // BGR颜色信息
    color_log << "\n=== 完整色块颜色信息 ===" << std::endl;
    color_log << "中心颜色 (BGR): [" << (int)center_color[0] << ", " 
              << (int)center_color[1] << ", " << (int)center_color[2] << "]" << std::endl;
    
    // 记录所有外围颜色（包括目标色块）
    color_log << "外围颜色列表 (共" << surround_colors.size() << "个):" << std::endl;
    for (size_t i = 0; i < surround_colors.size(); i++) {
        const auto& color = surround_colors[i];
        std::string is_target = (i == (size_t)target_index) ? " [TARGET]" : "";
        color_log << "  色块" << i << " (BGR): [" << (int)color[0] << ", " 
                  << (int)color[1] << ", " << (int)color[2] << "]" << is_target << std::endl;
    }
    
    // 目标色块信息
    if (target_index >= 0 && target_index < (int)surround_colors.size()) {
        cv::Scalar actual_target_color = surround_colors[target_index];
        color_log << "\n目标色块详细信息:" << std::endl;
        color_log << "  目标索引: " << target_index << std::endl;
        color_log << "  目标颜色 (BGR): [" << (int)actual_target_color[0] << ", " 
                  << (int)actual_target_color[1] << ", " << (int)actual_target_color[2] << "]" << std::endl;
    }
    
    // HSV颜色信息
    color_log << "\n=== HSV颜色空间分析 ===" << std::endl;
    
    // 中心颜色的HSV
    cv::Mat center_bgr(1, 1, CV_8UC3, center_color);
    cv::Mat center_hsv;
    cv::cvtColor(center_bgr, center_hsv, cv::COLOR_BGR2HSV);
    cv::Vec3b center_hsv_val = center_hsv.at<cv::Vec3b>(0, 0);
    color_log << "中心颜色 (HSV): H=" << (int)center_hsv_val[0] << "° S=" 
              << (int)center_hsv_val[1] << "% V=" << (int)center_hsv_val[2] << "%" << std::endl;
    
    // 所有外围色块的HSV
    color_log << "外围色块HSV:" << std::endl;
    for (size_t i = 0; i < surround_colors.size(); i++) {
        const auto& color = surround_colors[i];
        cv::Mat color_bgr(1, 1, CV_8UC3, color);
        cv::Mat color_hsv;
        cv::cvtColor(color_bgr, color_hsv, cv::COLOR_BGR2HSV);
        cv::Vec3b color_hsv_val = color_hsv.at<cv::Vec3b>(0, 0);
        
        std::string is_target = (i == (size_t)target_index) ? " [TARGET]" : "";
        color_log << "  色块" << i << " (HSV): H=" << (int)color_hsv_val[0] << "° S=" 
                  << (int)color_hsv_val[1] << "% V=" << (int)color_hsv_val[2] << "%" << is_target << std::endl;
    }
    
    // 颜色对比度分析（增强版）
    color_log << "\n=== 颜色对比度详细分析 ===" << std::endl;
    
    // 1. 中心颜色与所有外围颜色的对比
    color_log << "中心颜色与外围颜色对比:" << std::endl;
    for (size_t i = 0; i < surround_colors.size(); i++) {
        const auto& color = surround_colors[i];
        
        // 计算BGR空间的距离
        float bgr_distance = cv::norm(center_color - color);
        
        // 计算HSV空间的差异
        cv::Mat color_bgr(1, 1, CV_8UC3, color);
        cv::Mat color_hsv;
        cv::cvtColor(color_bgr, color_hsv, cv::COLOR_BGR2HSV);
        cv::Vec3b color_hsv_val = color_hsv.at<cv::Vec3b>(0, 0);
        
        float hue_diff = std::abs((int)center_hsv_val[0] - (int)color_hsv_val[0]);
        if (hue_diff > 90) hue_diff = 180 - hue_diff; // HSV圆形特性
        
        float saturation_diff = std::abs((int)center_hsv_val[1] - (int)color_hsv_val[1]);
        float value_diff = std::abs((int)center_hsv_val[2] - (int)color_hsv_val[2]);
        
        std::string is_target = (i == (size_t)target_index) ? " [TARGET]" : "";
        std::string similarity_note = "";
        
        // 判断相似度
        if (bgr_distance < 5.0f) {
            similarity_note = " (颜色几乎相同)";
        } else if (hue_diff < 5 && bgr_distance < 30) {
            similarity_note = " (非常相似)";
        } else if (hue_diff < 10 && bgr_distance < 50) {
            similarity_note = " (相似)";
        } else if (hue_diff > 60) {
            similarity_note = " (色调差异大)";
        }
        
        color_log << "  色块" << i << ": BGR距离=" << bgr_distance 
                  << ", 色调差异=" << hue_diff << "°"
                  << ", 饱和度差异=" << saturation_diff << "%"
                  << ", 亮度差异=" << value_diff << "%"
                  << is_target << similarity_note << std::endl;
    }
    
    // 2. 外围色块之间的对比
    color_log << "\n外围色块之间的对比:" << std::endl;
    for (size_t i = 0; i < surround_colors.size(); i++) {
        for (size_t j = i + 1; j < surround_colors.size(); j++) {
            const auto& color1 = surround_colors[i];
            const auto& color2 = surround_colors[j];
            
            float bgr_distance = cv::norm(color1 - color2);
            
            cv::Mat bgr1(1, 1, CV_8UC3, color1);
            cv::Mat bgr2(1, 1, CV_8UC3, color2);
            cv::Mat hsv1, hsv2;
            cv::cvtColor(bgr1, hsv1, cv::COLOR_BGR2HSV);
            cv::cvtColor(bgr2, hsv2, cv::COLOR_BGR2HSV);
            cv::Vec3b hsv_val1 = hsv1.at<cv::Vec3b>(0, 0);
            cv::Vec3b hsv_val2 = hsv2.at<cv::Vec3b>(0, 0);
            
            float hue_diff = std::abs((int)hsv_val1[0] - (int)hsv_val2[0]);
            if (hue_diff > 90) hue_diff = 180 - hue_diff;
            
            // 如果两个色块颜色接近，特别标记
            if (bgr_distance < 20.0f) {
                std::string note1 = (i == (size_t)target_index) ? "[目标]" : "";
                std::string note2 = (j == (size_t)target_index) ? "[目标]" : "";
                color_log << "  色块" << i << note1 << " 与 色块" << j << note2 
                          << " 非常相似 (BGR距离=" << bgr_distance 
                          << ", 色调差异=" << hue_diff << "°)" << std::endl;
            }
        }
    }
    
    // 3. 目标色块与其他色块的特别比较
    if (target_index >= 0 && target_index < (int)surround_colors.size()) {
        cv::Scalar target_color = surround_colors[target_index];
        color_log << "\n目标色块与其他色块的特别比较:" << std::endl;
        
        // 目标色块与中心色块的相似度
        float target_center_bgr_dist = cv::norm(target_color - center_color);
        color_log << "  目标色块-中心色块 BGR距离: " << target_center_bgr_dist 
                  << (target_center_bgr_dist < 10 ? " (非常接近)" : "") << std::endl;
        
        // 目标色块与每个非目标外围色块的相似度
        for (size_t i = 0; i < surround_colors.size(); i++) {
            if (i == (size_t)target_index) continue;
            
            const auto& color = surround_colors[i];
            float bgr_dist = cv::norm(target_color - color);
            
            if (bgr_dist < 30.0f) {
                color_log << "  目标色块与色块" << i << " 非常相似 (BGR距离=" 
                          << bgr_dist << ")" << std::endl;
            }
        }
    }
    
    // 4. 颜色分布特征统计
    color_log << "\n=== 颜色分布特征统计 ===" << std::endl;
    
    // 计算平均亮度和饱和度
    float avg_value = center_hsv_val[2];
    float avg_saturation = center_hsv_val[1];
    int low_value_count = (center_hsv_val[2] < 50) ? 1 : 0;
    int low_saturation_count = (center_hsv_val[1] < 50) ? 1 : 0;
    
    for (size_t i = 0; i < surround_colors.size(); i++) {
        const auto& color = surround_colors[i];
        cv::Mat color_bgr(1, 1, CV_8UC3, color);
        cv::Mat color_hsv;
        cv::cvtColor(color_bgr, color_hsv, cv::COLOR_BGR2HSV);
        cv::Vec3b hsv_val = color_hsv.at<cv::Vec3b>(0, 0);
        
        avg_value += hsv_val[2];
        avg_saturation += hsv_val[1];
        
        if (hsv_val[2] < 50) low_value_count++;
        if (hsv_val[1] < 50) low_saturation_count++;
    }
    
    avg_value /= (surround_colors.size() + 1);
    avg_saturation /= (surround_colors.size() + 1);
    
    color_log << "平均亮度: " << avg_value << "%" << std::endl;
    color_log << "平均饱和度: " << avg_saturation << "%" << std::endl;
    color_log << "低亮度色块(<50%): " << low_value_count << "/" << (surround_colors.size() + 1) << std::endl;
    color_log << "低饱和度色块(<50%): " << low_saturation_count << "/" << (surround_colors.size() + 1) << std::endl;
    
    // 5. 潜在问题分析
    color_log << "\n=== 潜在问题分析 ===" << std::endl;
    
    bool has_potential_issues = false;
    
    // 检查是否有多个色块与目标色块颜色相似
    if (target_index >= 0 && target_index < (int)surround_colors.size()) {
        cv::Scalar target_color = surround_colors[target_index];
        int similar_count = 0;
        
        for (size_t i = 0; i < surround_colors.size(); i++) {
            if (i == (size_t)target_index) continue;
            
            const auto& color = surround_colors[i];
            float bgr_dist = cv::norm(target_color - color);
            
            if (bgr_dist < 30.0f) {
                similar_count++;
                color_log << "⚠️  色块" << i << " 与目标色块颜色相似 (BGR距离=" 
                          << bgr_dist << ")" << std::endl;
            }
        }
        
        if (similar_count > 0) {
            has_potential_issues = true;
            color_log << "警告: 有 " << similar_count << " 个非目标色块与目标色块颜色相似" << std::endl;
        }
    }
    
    // 检查中心与目标是否太相似
    float center_target_dist = cv::norm(center_color - target_color);
    if (center_target_dist < 10.0f) {
        has_potential_issues = true;
        color_log << "⚠️  中心颜色与目标颜色过于相似 (BGR距离=" << center_target_dist << ")" << std::endl;
    }
    
    // 检查低亮度问题
    if (static_cast<size_t>(low_value_count) > (surround_colors.size() + 1) / 2) {
        has_potential_issues = true;
        color_log << "⚠️  超过一半的色块亮度低于50%" << std::endl;
    }
    
    if (!has_potential_issues) {
        color_log << "未发现明显的颜色配置问题" << std::endl;
    }
    
    color_log << "错误帧文件: error_target_" << target_id << "_t" << static_cast<int>(timestamp) << ".jpg" << std::endl;
    
    // 3. 记录到CSV文件（包含完整颜色信息）
    detailed_log << target_id << ","
                 << timestamp << ","
                 << (accuracy < 90.0f ? "FAILURE" : "SUCCESS") << ","
                 << accuracy << ","
                 << test_time << ","
                 << (int)center_color[0] << "," << (int)center_color[1] << "," << (int)center_color[2] << ","
                 << (int)target_color[0] << "," << (int)target_color[1] << "," << (int)target_color[2] << ","
                 << target_index << ",";
    
    // 记录所有外围颜色
    for (size_t i = 0; i < 5; i++) {
        if (i < surround_colors.size()) {
            const auto& color = surround_colors[i];
            detailed_log << (int)color[0] << "," << (int)color[1] << "," << (int)color[2] << ",";
        } else {
            detailed_log << "-1,-1,-1,"; // 占位符
        }
    }
    
    detailed_log << total_frames << ","
                 << success_frames << ","
                 << "error_target_" << target_id << "_t" << static_cast<int>(timestamp) << ".jpg"
                 << std::endl;
    
    // 额外保存一个详细的JSON格式日志，便于分析
    std::ofstream json_log("detailed_color_analysis_" + std::to_string(target_id) + ".json", std::ios::app);
    if (json_log.is_open()) {
        json_log << "{\n";
        json_log << "  \"target_id\": " << target_id << ",\n";
        json_log << "  \"timestamp\": " << timestamp << ",\n";
        json_log << "  \"accuracy\": " << accuracy << ",\n";
        json_log << "  \"target_index\": " << target_index << ",\n";
        
        // 中心颜色
        json_log << "  \"center_color\": {\"b\": " << (int)center_color[0] 
                 << ", \"g\": " << (int)center_color[1] 
                 << ", \"r\": " << (int)center_color[2] 
                 << ", \"h\": " << (int)center_hsv_val[0]
                 << ", \"s\": " << (int)center_hsv_val[1]
                 << ", \"v\": " << (int)center_hsv_val[2] << "},\n";
        
        // 所有外围颜色
        json_log << "  \"surround_colors\": [\n";
        for (size_t i = 0; i < surround_colors.size(); i++) {
            const auto& color = surround_colors[i];
            cv::Mat color_bgr(1, 1, CV_8UC3, color);
            cv::Mat color_hsv;
            cv::cvtColor(color_bgr, color_hsv, cv::COLOR_BGR2HSV);
            cv::Vec3b hsv_val = color_hsv.at<cv::Vec3b>(0, 0);
            
            json_log << "    {\"index\": " << i 
                     << ", \"is_target\": " << (i == (size_t)target_index ? "true" : "false")
                     << ", \"bgr\": [" << (int)color[0] << ", " << (int)color[1] << ", " << (int)color[2] << "]"
                     << ", \"hsv\": [" << (int)hsv_val[0] << ", " << (int)hsv_val[1] << ", " << (int)hsv_val[2] << "]"
                     << ", \"distance_to_center\": " << cv::norm(center_color - color);
            
            if (i == (size_t)target_index) {
                json_log << ", \"distance_to_target\": 0";
            } else if (target_index >= 0 && target_index < (int)surround_colors.size()) {
                cv::Scalar target_color = surround_colors[target_index];
                json_log << ", \"distance_to_target\": " << cv::norm(color - target_color);
            }
            
            json_log << "}";
            if (i < surround_colors.size() - 1) json_log << ",";
            json_log << "\n";
        }
        json_log << "  ]\n";
        json_log << "}\n";
        json_log.close();
    }
    
    // 刷新缓冲区
    failure_log.flush();
    color_log.flush();
    detailed_log.flush();
    
    std::cout << "✅ 已记录故障靶子 " << target_id << " 的完整颜色信息" << std::endl;
    std::cout << "   - 详细分析已保存到 JSON 文件: detailed_color_analysis_" << target_id << ".json" << std::endl;
}

/**
 * @brief 生成颜色分析报告（增强版）
 */
void generate_color_analysis_report(const std::vector<TargetColorInfo>& problematic_infos) {
    std::ofstream report("color_analysis_report.txt");
    
    if (!report.is_open()) {
        std::cerr << "无法创建颜色分析报告文件" << std::endl;
        return;
    }
    
    time_t now = time(nullptr);
    std::string time_str = std::ctime(&now);
    time_str = time_str.substr(0, time_str.length() - 1);
    
    report << "=== 故障颜色组合分析报告 ===" << std::endl;
    report << "生成时间: " << time_str << std::endl;
    report << "故障靶子总数: " << problematic_infos.size() << std::endl;
    report << "=====================================" << std::endl;
    
    if (problematic_infos.empty()) {
        report << "没有发现故障靶子" << std::endl;
        return;
    }
    
    // 按准确率排序
    auto sorted_infos = problematic_infos;
    std::sort(sorted_infos.begin(), sorted_infos.end(), 
              [](const TargetColorInfo& a, const TargetColorInfo& b) {
                  return a.accuracy < b.accuracy;
              });
    
    // 输出最差的几个靶子
    report << "\n=== 最差的5个靶子 ===" << std::endl;
    int count = std::min(5, (int)sorted_infos.size());
    for (int i = 0; i < count; i++) {
        const auto& info = sorted_infos[i];
        report << i+1 << ". 靶子ID: " << info.target_id 
               << ", 准确率: " << info.accuracy << "%"
               << ", 目标索引: " << info.target_index  // 修改这里：使用 target_index
               << ", 测试帧数: " << info.test_frames 
               << ", 错误帧: " << info.error_frame_file << std::endl;
        
        // 输出颜色信息
        report << "   中心颜色 (BGR): [" << (int)info.center_color_bgr[0] << ", "
               << (int)info.center_color_bgr[1] << ", " << (int)info.center_color_bgr[2] << "]" << std::endl;
        report << "   目标颜色 (BGR): [" << (int)info.target_color_bgr[0] << ", "
               << (int)info.target_color_bgr[1] << ", " << (int)info.target_color_bgr[2] << "]" << std::endl;
    }
    
    // 分析准确率分布
    report << "\n=== 准确率分布 ===" << std::endl;
    std::vector<int> accuracy_ranges = {0, 20, 40, 60, 80};
    std::vector<int> counts(accuracy_ranges.size(), 0);
    
    for (const auto& info : sorted_infos) {
        for (size_t i = 0; i < accuracy_ranges.size(); i++) {
            if (info.accuracy < accuracy_ranges[i] + 20) {
                counts[i]++;
                break;
            }
        }
    }
    
    for (size_t i = 0; i < accuracy_ranges.size(); i++) {
        report << accuracy_ranges[i] << "-" << (accuracy_ranges[i] + 20) << "%: " 
               << counts[i] << " 个 (" 
               << (100.0f * counts[i] / sorted_infos.size()) << "%)" << std::endl;
    }
    
    // 分析目标索引分布
    report << "\n=== 目标索引分布 ===" << std::endl;
    std::map<int, int> index_counts;
    for (const auto& info : sorted_infos) {
        if (info.target_index >= 0) {
            index_counts[info.target_index]++;
        }
    }
    
    for (const auto& pair : index_counts) {
        report << "索引 " << pair.first << ": " << pair.second << " 个 ("
               << (100.0f * pair.second / sorted_infos.size()) << "%)" << std::endl;
    }
    
    // 分析颜色相似性问题
    report << "\n=== 颜色相似性分析 ===" << std::endl;
    int similar_colors_count = 0;
    int black_colors_count = 0;
    
    for (const auto& info : sorted_infos) {
        // 检查中心颜色与目标颜色是否相似
        float bgr_distance = cv::norm(info.center_color_bgr - info.target_color_bgr);
        if (bgr_distance < 50.0f) {
            similar_colors_count++;
            report << "靶子ID " << info.target_id << ": 中心与目标颜色非常相似 (BGR距离=" 
                   << bgr_distance << ")" << std::endl;
        }
        
        // 检查是否有黑色或低亮度颜色
        cv::Mat center_bgr(1, 1, CV_8UC3, info.center_color_bgr);
        cv::Mat center_hsv;
        cv::cvtColor(center_bgr, center_hsv, cv::COLOR_BGR2HSV);
        cv::Vec3b center_hsv_val = center_hsv.at<cv::Vec3b>(0, 0);
        
        if (center_hsv_val[2] < 50) {  // 亮度低于50
            black_colors_count++;
            report << "靶子ID " << info.target_id << ": 中心颜色亮度低 (V=" 
                   << (int)center_hsv_val[2] << "%)" << std::endl;
        }
    }
    
    report << "\n统计摘要:" << std::endl;
    report << "- 中心与目标颜色相似的靶子: " << similar_colors_count << " 个" << std::endl;
    report << "- 中心颜色亮度低的靶子: " << black_colors_count << " 个" << std::endl;
    
    // 输出所有故障靶子的ID
    report << "\n=== 所有故障靶子ID ===" << std::endl;
    for (size_t i = 0; i < sorted_infos.size(); i++) {
        report << sorted_infos[i].target_id;
        if ((i + 1) % 10 == 0) report << std::endl;
        else if (i < sorted_infos.size() - 1) report << ", ";
    }
    report << std::endl;
    
    // 建议检查项
    report << "\n=== 改进建议 ===" << std::endl;
    report << "基于分析结果，建议：" << std::endl;
    report << "1. 检查颜色相似性问题：" << std::endl;
    report << "   - 中心颜色与目标颜色过于相似可能导致识别困难" << std::endl;
    report << "   - 考虑增加颜色对比度阈值" << std::endl;
    report << "2. 亮度问题：" << std::endl;
    report << "   - 低亮度颜色在HSV空间中可能难以区分" << std::endl;
    report << "   - 考虑过滤亮度过低的颜色" << std::endl;
    report << "3. 目标索引分布：" << std::endl;
    report << "   - 某些位置的目标可能更容易识别错误" << std::endl;
    report << "   - 检查特定位置的颜色配置" << std::endl;
    report << "4. 算法优化：" << std::endl;
    report << "   - 在HSV空间中增加色调权重" << std::endl;
    report << "   - 添加颜色对比度检查" << std::endl;
    report << "   - 考虑使用更高级的颜色匹配算法" << std::endl;
    
    report.close();
}


TrainingFrameGenerator::AngularVelocityFunction energy_mechanism_velocity_generator() {
    // 随机数生成器
    static std::random_device rd;
    static std::mt19937 gen(rd());
    
    // 参数范围
    std::uniform_real_distribution<float> a_dist(0.780f, 1.045f);  // a ∈ [0.780, 1.045]
    std::uniform_real_distribution<float> omega_dist(1.884f, 2.000f); // ω ∈ [1.884, 2.000]
    
    // 随机生成参数
    float a = a_dist(gen);
    float omega = omega_dist(gen);
    float b = 2.090f - a;  // b = 2.090 - a
    
    std::cout << "[Energy Mechanism] Generated parameters: "
              << "a = " << a << ", ω = " << omega 
              << ", b = " << b << ", spd(t) = " << a << " * sin(" << omega << " * t) + " << b 
              << std::endl;
    
    return [a, omega, b](float t) -> float {
        return a * std::sin(omega * t) + b;
    };
}
//计算检测到的目标位置和实际位置的误差，如果低于某个阈值则认为检测成功
bool calculate_position_error(const cv::Point2f& detected, const cv::Point2f& actual) {
    float error = cv::norm(detected - actual);
    if(error < 10.0f) {
        std::cout << "Target Match at an error of " << error << std::endl;
        return true;
   
    } else {
        std::cout << "Match failed" << std::endl;
        // TrackerConfig config;
        // config.color_similarity_threshold = 30.0f; // 放宽颜色阈值
        // set_config(config);
        return false;
    }
}

bool calculate_position_error(const float& detected, float& actual) {
    float error = cv::norm(detected - actual);
    if(error < 10.0f) {
        std::cout << "Target Match at an error of " << error << std::endl;
        return true;
    } else {
        std::cout << "Match failed" << std::endl;
        // TrackerConfig config;
        // config.color_similarity_threshold = 30.0f; // 放宽颜色阈值
        // set_config(config);
        return false;
    }
}

/**
 * @brief 测试随机五角星旋转模式（带完整颜色信息记录）
 */
void test_pentagon_rotation(TrainingFrameGenerator& generator) {
    std::cout << "\n=== Pentagon Rotation Test ===" << std::endl;
    
    // 设置训练模式为五角星旋转
    float angular_speed = 0.5f;
    TrainingFrameGenerator::AngularVelocityFunction angular_velocity_func = 
        energy_mechanism_velocity_generator();
    generator.set_training_mode(TrainingFrameGenerator::MODE_PENTAGON_ROTATION, 1, 1, angular_velocity_func);
    
    // 状态变量
    bool show_markers = true;  // 默认显示标记
    
    // 使用简化的性能监视器
    PerformanceMonitor perf_monitor;
    
    cv::namedWindow("Pentagon Rotation", cv::WINDOW_AUTOSIZE);
    
    // 创建故障记录文件
    std::ofstream failure_log("failure_log.txt", std::ios::app);
    std::ofstream color_log("color_combinations.txt", std::ios::app);
    
    // 创建CSV格式的详细日志
    std::ofstream detailed_log("detailed_failure_log.csv", std::ios::app);
    
    // 如果是新文件，写入CSV表头
    if (detailed_log.tellp() == 0) {
        detailed_log << "target_id,timestamp,failure_type,accuracy,test_time,"
                     << "center_color_b,center_color_g,center_color_r,"
                     << "target_color_b,target_color_g,target_color_r,"
                     << "target_index,"
                     << "surround_color1_b,surround_color1_g,surround_color1_r,"
                     << "surround_color2_b,surround_color2_g,surround_color2_r,"
                     << "surround_color3_b,surround_color3_g,surround_color3_r,"
                     << "surround_color4_b,surround_color4_g,surround_color4_r,"
                     << "surround_color5_b,surround_color5_g,surround_color5_r,"
                     << "total_frames,success_frames,error_frame_file\n";
    }
    
    std::cout << "\nTest started. Pentagon is auto-rotating at center." << std::endl;
    std::cout << "Target markers are ON (default). Press T to toggle." << std::endl;
    std::cout << "Press SPACE to pause/resume rotation." << std::endl;
    std::cout << "Press D to手动记录当前靶子信息" << std::endl;
    std::cout << "Press S to显示统计摘要" << std::endl;
    std::cout << "故障记录将保存到: failure_log.txt, color_combinations.txt, detailed_failure_log.csv" << std::endl;
    
    TargetTracker tracker;
     // 配置BGR暗色检测阈值
    tracker.set_dark_brightness_threshold(60); // 默认50，可以调整
    
    // 启用调试模式
    tracker.set_debug_mode(true);
    
    // 启用角度约束
    tracker.enable_angle_constraint(true);
    tracker.set_angle_prediction_threshold(5.0f);
    
    // 统计变量
    int total_frames = 0;                     // 总帧数
    int current_session_frames = 0;           // 当前靶子测试帧数
    int current_session_success = 0;          // 当前靶子成功帧数
    int total_targets_generated = 1;          // 总共生成的靶子数量（从1开始）
    int problematic_targets = 0;              // 出现故障的靶子数量（准确率<90%）
    
    // 定时切换相关变量
    float current_session_start_time = 0.0f;  // 当前靶子开始时间
    bool auto_switch_enabled = true;          // 自动切换开关（默认开启）
    float success_duration_needed = 3.0f;     // 成功需要持续的时间（秒）
    float fail_duration_limit = 3.0f;         // 失败最多测试时间（秒）
    
    // 颜色信息记录
    std::vector<TargetColorInfo> problematic_color_infos;
    int next_target_id = 1;
    
    // 获取第一帧以初始化时间
    auto first_frame_data = generator.get_next_frame();
    current_session_start_time = first_frame_data.timestamp;
    
    // 处理第一帧
    auto tracker_result = tracker.process_frame(first_frame_data.frame);
    total_frames++;
    current_session_frames++;
    
    if(calculate_position_error(tracker_result.target_center, first_frame_data.target_position)){
        current_session_success++;
    }
    
    while (true) {
        // 获取训练帧
        auto frame_data = generator.get_next_frame();
        auto tracker_result = tracker.process_frame(frame_data.frame);
        
        // std::cout << "Tracker found: " << (tracker_result.found ? "YES" : "NO") 
        //           << ", Position: (" << tracker_result.target_center.x << ", " << tracker_result.target_center.y << ")"
        //           << ", Distance: " << tracker_result.distance
        //           << ", Angle: " << tracker_result.angle << " degrees"
        //           << std::endl;
        
        // 更新帧数计数
        total_frames++;
        current_session_frames++;
        
        // 检查识别结果
        // bool is_correct = false;
        if (tracker_result.found) {
            float error = cv::norm(tracker_result.target_center - frame_data.target_position);
            if (error < 10.0f) {
                current_session_success++;
                // is_correct = true;
            } else {
                // 保存错误帧
                std::string error_filename = "error_target_" + std::to_string(next_target_id) + 
                                            "_t" + std::to_string(static_cast<int>(frame_data.timestamp)) + 
                                            ".jpg";
                imwrite(error_filename, frame_data.frame);
                
                // std::cout << "❌ 识别错误！已保存帧到: " << error_filename << std::endl;
                // std::cout << "   期望位置: (" << frame_data.target_position.x << ", " 
                //           << frame_data.target_position.y << ")" << std::endl;
                // std::cout << "   检测位置: (" << tracker_result.target_center.x << ", " 
                //           << tracker_result.target_center.y << ")" << std::endl;
            }
        } else {
            // std::cout << "❌ 未检测到目标！" << std::endl;
        }
        
        // 更新FPS
        float fps = perf_monitor.tick();
        
        // 计算当前靶子运行时间和准确率
        float current_session_time = frame_data.timestamp - current_session_start_time;
        float current_accuracy = (current_session_frames > 0) ? 
            (100.0f * current_session_success / current_session_frames) : 0.0f;
        
        // 检查是否需要自动切换
        if (auto_switch_enabled && current_session_time > 0.5f) { // 至少运行0.5秒才开始检查
            bool should_switch = false;
            // bool is_failure = false;
            
            if (current_accuracy >= 90.0f && current_session_time >= success_duration_needed) {
                // 准确率≥90%且已运行3秒，成功切换
                std::cout << "✅ 当前靶子准确率" << current_accuracy << "%，已稳定运行" 
                          << current_session_time << "秒，准备切换..." << std::endl;
                should_switch = true;
            } else if (current_session_time >= fail_duration_limit) {
                // 已运行5秒但准确率<90%，失败切换
                std::cout << "⚠️  当前靶子准确率" << current_accuracy << "%，已测试" 
                          << current_session_time << "秒，强制切换..." << std::endl;
                problematic_targets++;
                should_switch = true;
                // is_failure = true;
                
                // 获取当前靶子的颜色信息并记录故障
                TargetSim& target_sim = generator.get_target_sim();
                record_failure_info_with_colors(target_sim, next_target_id, frame_data.timestamp, 
                                              current_accuracy, current_session_time, 
                                              current_session_frames, current_session_success,
                                              failure_log, color_log, detailed_log);
                
                // 保存到向量用于分析
                TargetColorInfo info;
                info.target_id = next_target_id;
                info.timestamp = std::to_string(frame_data.timestamp);
                info.accuracy = current_accuracy;
                info.test_frames = current_session_frames;
                info.error_frame_file = "error_target_" + std::to_string(next_target_id) + 
                                       "_t" + std::to_string(static_cast<int>(frame_data.timestamp)) + 
                                       ".jpg";
                
                // 保存颜色信息
                info.center_color_bgr = target_sim.get_center_color();
                info.target_index = target_sim.get_target_index();
                if (info.target_index >= 0 && info.target_index < (int)target_sim.get_surround_colors().size()) {
                    info.target_color_bgr = target_sim.get_surround_colors()[info.target_index];
                }
                info.surround_colors_bgr = target_sim.get_surround_colors();
                
                problematic_color_infos.push_back(info);
            }
            
            if (should_switch) {
                next_target_id++;
                
                // 执行切换
                total_targets_generated++;
                generator.regenerate_pentagon(generator.get_target_sim_center()); // 随机位置
                
                std::cout << "\n=== 自动切换 ===" << std::endl;
                std::cout << "已生成第 " << total_targets_generated << " 个靶子 (ID: " << next_target_id << ")" << std::endl;
                std::cout << "当前故障率: " << (100.0f * problematic_targets / (total_targets_generated - 1)) << "%" << std::endl;
                
                // 重置当前会话统计
                current_session_frames = 0;
                current_session_success = 0;
                current_session_start_time = frame_data.timestamp;
                
                continue; // 跳过本次循环的显示，直接处理下一帧
            }
        }
        
        // 创建显示图像
        cv::Mat display = frame_data.frame.clone();
        
        // 获取当前中心位置
        cv::Point2f center = generator.get_current_pentagon_center();
        
        // === 如果显示标记为true，绘制所有标记 ===
        if (show_markers) {
            // 1. 绘制目标色块标记（大红点）
            cv::circle(display, frame_data.target_position, 15, cv::Scalar(0, 0, 255), -1);
            cv::circle(display, frame_data.target_position, 18, cv::Scalar(255, 255, 255), 3);
            
            // 2. 绘制中心色块标记（小蓝点）
            cv::circle(display, center, 8, cv::Scalar(255, 0, 0), -1);
            cv::circle(display, center, 11, cv::Scalar(255, 255, 255), 2);
            
            // 3. 绘制从中心到目标的连线（绿线）
            cv::line(display, center, frame_data.target_position, 
                    cv::Scalar(0, 255, 0), 2);
            
            // 4. 在目标旁边显示"TARGET"标签
            cv::putText(display, "TARGET", 
                       cv::Point(frame_data.target_position.x + 25, frame_data.target_position.y - 10),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 1);
            
            // 5. 在中心旁边显示"CENTER"标签
            cv::putText(display, "CENTER", 
                       cv::Point(center.x + 15, center.y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
        }
        
        // === 显示性能信息 ===
        // FPS信息（带颜色编码）
        std::string fps_text = "FPS: " + std::to_string(int(fps));
        cv::Scalar fps_color = cv::Scalar(0, 255, 0); // 默认绿色
        
        if (fps < 30.0f) {
            fps_color = cv::Scalar(0, 0, 255); // 红色（低帧率）
        } else if (fps < 50.0f) {
            fps_color = cv::Scalar(0, 165, 255); // 橙色
        }
        
        cv::putText(display, fps_text, 
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                   fps_color, 1);
        
        // 当前靶子统计信息
        std::string accuracy_text = "Current Acc: " + std::to_string(current_accuracy).substr(0, 5) + "%";
        cv::Scalar accuracy_color = (current_accuracy >= 90.0f) ? cv::Scalar(0, 255, 0) : 
                                   (current_accuracy >= 70.0f) ? cv::Scalar(0, 165, 255) : 
                                   cv::Scalar(0, 0, 255);
        
        cv::putText(display, accuracy_text, 
                   cv::Point(10, 55), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                   accuracy_color, 1);
        
        // 当前靶子运行时间
        std::string time_text = "Time: " + std::to_string(current_session_time).substr(0, 4) + "s";
        cv::putText(display, time_text, 
                   cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                   cv::Scalar(255, 255, 0), 1);
        
        // 帧数
        cv::putText(display, "Frame: " + std::to_string(total_frames), 
                   cv::Point(10, 105), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                   cv::Scalar(255, 255, 0), 1);
        
        // 全局统计
        std::string global_text = "Targets: " + std::to_string(total_targets_generated) + 
                                 " | Problematic: " + std::to_string(problematic_targets);
        cv::Scalar global_color = (problematic_targets == 0) ? cv::Scalar(0, 255, 0) : 
                                 (problematic_targets * 1.0 / total_targets_generated < 0.05) ? 
                                 cv::Scalar(0, 165, 255) : cv::Scalar(0, 0, 255);
        
        cv::putText(display, global_text, 
                   cv::Point(10, 130), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                   global_color, 1);
        
        // 靶子ID
        cv::putText(display, "Target ID: " + std::to_string(next_target_id), 
                   cv::Point(10, 155), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                   cv::Scalar(200, 200, 0), 1);
        
        // 状态
        std::string status_text = "Status: ";
        status_text += generator.is_paused() ? "PAUSED" : "RUNNING";
        status_text += auto_switch_enabled ? " | AUTO" : " | MANUAL";
        cv::Scalar status_color = generator.is_paused() ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
        cv::putText(display, status_text, 
                   cv::Point(10, 180), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                   status_color, 1);
        
        // 显示
        cv::imshow("Pentagon Rotation", display);
        
        // 处理按键
        int key = cv::waitKey(10);
        if (key == 27) { // ESC
            break;
        } else if (key == 32) { // SPACE 暂停/继续
            if (generator.is_paused()) {
                generator.resume();
                std::cout << "Rotation RESUMED at time: " << generator.get_current_time() << "s" << std::endl;
            } else {
                generator.pause();
                std::cout << "Rotation PAUSED at time: " << generator.get_current_time() << "s" << std::endl;
            }
        } else if (key == 't' || key == 'T') { // T键切换标记显示
            show_markers = !show_markers;
            std::cout << "Target markers: " << (show_markers ? "ON" : "OFF") << std::endl;
        } else if (key == 'a' || key == 'A') { // A键切换自动/手动模式
            auto_switch_enabled = !auto_switch_enabled;
            std::cout << "Auto-switch: " << (auto_switch_enabled ? "ENABLED" : "DISABLED") << std::endl;
        } else if (key == 'd' || key == 'D') { // D键手动记录当前靶子信息
            std::cout << "\n=== 手动记录当前靶子信息 ===" << std::endl;
            TargetSim& target_sim = generator.get_target_sim();
            record_failure_info_with_colors(target_sim, next_target_id, frame_data.timestamp, 
                                          current_accuracy, current_session_time, 
                                          current_session_frames, current_session_success,
                                          failure_log, color_log, detailed_log);
            std::cout << "已记录当前靶子信息 (ID: " << next_target_id << ")" << std::endl;
        } else if (key == 'r' || key == 'R' || key == 'c' || key == 'C' || key == '0') { 
            // R/C/0键：手动生成新靶子
            
            // 1. 首先统计当前靶子
            if (current_session_frames > 0) {
                float accuracy_rate = current_accuracy / 100.0f;
                
                std::cout << "\n=== 当前靶子统计 ===" << std::endl;
                std::cout << "测试帧数: " << current_session_frames << std::endl;
                std::cout << "成功帧数: " << current_session_success << std::endl;
                std::cout << "运行时间: " << current_session_time << "秒" << std::endl;
                std::cout << "识别准确率: " << current_accuracy << "%" << std::endl;
                
                // 如果准确率低于90%，计入故障统计并记录信息
                if (accuracy_rate < 0.9f) {
                    problematic_targets++;
                    std::cout << "⚠️  本次追踪失败，已计入故障统计" << std::endl;
                    
                    // 获取颜色信息并记录
                    TargetSim& target_sim = generator.get_target_sim();
                    record_failure_info_with_colors(target_sim, next_target_id, frame_data.timestamp, 
                                                  current_accuracy, current_session_time, 
                                                  current_session_frames, current_session_success,
                                                  failure_log, color_log, detailed_log);
                    
                    // 保存到向量用于分析
                    TargetColorInfo info;
                    info.target_id = next_target_id;
                    info.timestamp = std::to_string(frame_data.timestamp);
                    info.accuracy = current_accuracy;
                    info.test_frames = current_session_frames;
                    info.error_frame_file = "error_target_" + std::to_string(next_target_id) + 
                                           "_t" + std::to_string(static_cast<int>(frame_data.timestamp)) + 
                                           ".jpg";
                    
                    // 保存颜色信息
                    info.center_color_bgr = target_sim.get_center_color();
                    info.target_index = target_sim.get_target_index();
                    if (info.target_index >= 0 && info.target_index < (int)target_sim.get_surround_colors().size()) {
                        info.target_color_bgr = target_sim.get_surround_colors()[info.target_index];
                    }
                    info.surround_colors_bgr = target_sim.get_surround_colors();
                    
                    problematic_color_infos.push_back(info);
                } else {
                    std::cout << "✅  本次追踪成功" << std::endl;
                }
            }
            
            // 2. 生成新靶子
            next_target_id++;
            total_targets_generated++;
            
            if (key == 'r' || key == 'R') { // R键生成随机位置的新靶子
                generator.regenerate_pentagon(cv::Point2f(-1, -1)); // 随机位置
                std::cout << "Generated new pentagon at random position" << std::endl;
            } else if (key == 'c' || key == 'C') { // C键生成中心位置的新靶子
                generator.regenerate_pentagon(generator.get_target_sim_center());
                std::cout << "Generated new pentagon at center" << std::endl;
            } else if (key == '0') { // 0键重置角度和时间
                generator.reset();
                perf_monitor.reset();
                std::cout << "Training reset (time = 0, angle = 0)" << std::endl;
            }
            
            std::cout << "已生成第 " << total_targets_generated << " 个靶子 (ID: " << next_target_id << ")" << std::endl;
            
            // 3. 重置当前会话统计
            current_session_frames = 0;
            current_session_success = 0;
            current_session_start_time = frame_data.timestamp;
            
            std::cout << "已重置统计计数器" << std::endl;
            
        } else if (key == '+') { // +键增加旋转速度
            angular_speed += 0.1f;
            generator.set_training_mode(TrainingFrameGenerator::MODE_PENTAGON_ROTATION, angular_speed);
            std::cout << "Rotation speed increased to: " << angular_speed << " rad/s" << std::endl;
        } else if (key == '-') { // -键减少旋转速度
            angular_speed = std::max(0.1f, angular_speed - 0.1f);
            generator.set_training_mode(TrainingFrameGenerator::MODE_PENTAGON_ROTATION, angular_speed);
            std::cout << "Rotation speed decreased to: " << angular_speed << " rad/s" << std::endl;
        } else if (key == 's' || key == 'S') { // S键显示统计摘要
            std::cout << "\n=== 当前统计摘要 ===" << std::endl;
            std::cout << "总靶子数: " << total_targets_generated << std::endl;
            std::cout << "故障靶子数: " << problematic_targets << std::endl;
            std::cout << "故障率: " << (100.0f * problematic_targets / total_targets_generated) << "%" << std::endl;
            std::cout << "当前靶子ID: " << next_target_id << std::endl;
            std::cout << "已记录故障信息数量: " << problematic_color_infos.size() << std::endl;
        }
    }
    
    // 关闭日志文件
    failure_log.close();
    color_log.close();
    detailed_log.close();
    
    cv::destroyAllWindows();
    
    // ====== 测试结束时的性能总结 ======
    std::cout << "\n========== Test Summary ==========" << std::endl;
    std::cout << "总测试帧数: " << total_frames << std::endl;
    std::cout << "Average FPS: " << perf_monitor.get_average_fps() << std::endl;
    std::cout << "Minimum FPS: " << perf_monitor.get_min_fps() << std::endl;
    std::cout << "Maximum FPS: " << perf_monitor.get_max_fps() << std::endl;
    std::cout << "===================================" << std::endl;
    
    // 故障率统计
    if (total_targets_generated > 0) {
        float failure_probability = 100.0f * problematic_targets / total_targets_generated;
        
        std::cout << "\n=== 故障概率统计 ===" << std::endl;
        std::cout << "总共生成靶子数量: " << total_targets_generated << std::endl;
        std::cout << "出现故障的靶子数量: " << problematic_targets << std::endl;
        std::cout << "故障概率: " << failure_probability << "%" << std::endl;
        
        // 生成故障颜色组合分析报告
        if (!problematic_color_infos.empty()) {
            generate_color_analysis_report(problematic_color_infos);
        }
    }
    
    std::cout << "\n故障信息已保存到:" << std::endl;
    std::cout << "- failure_log.txt: 故障摘要" << std::endl;
    std::cout << "- color_combinations.txt: 颜色组合信息" << std::endl;
    std::cout << "- detailed_failure_log.csv: 详细CSV日志" << std::endl;
    
    if (!problematic_color_infos.empty()) {
        std::cout << "\n检测到 " << problematic_color_infos.size() << " 个有问题的颜色组合" << std::endl;
        std::cout << "已生成颜色分析报告: color_analysis_report.txt" << std::endl;
    }
    
    std::cout << "\nTest completed." << std::endl;
}


void test_pic() {
    cv::Mat img = cv::imread("error_frame.jpg");
    if(img.empty()) {
        std::cout << "ERROR: Cannot load image!" << std::endl;
        return;
    }
    
    std::cout << "=== IMAGE BASICS ===" << std::endl;
    std::cout << "Size: " << img.cols << "x" << img.rows << std::endl;
    std::cout << "Type: " << img.type() << " (CV_8UC3=" << CV_8UC3 << ")" << std::endl;
    
    TargetTracker tracker;
    // auto result = tracker.process_frame(img);
    // std::cout << "Tracker found: " << (result.found ? "YES" : "NO") 
    //           << ", Position: (" << result.target_center.x << ", " << result.target_center.y << ")"
    //           << ", Distance: " << result.distance
    //           << ", Angle: " << result.angle << " degrees"
    //           << std::endl;
}

// ==================== 参数调优函数 ====================

/**
 * @brief 测试特定参数组合
 */
float test_parameter_config(TargetTracker& tracker, 
                           TrainingFrameGenerator& generator,
                           int test_frames = 360) {  // 测试一圈完整的旋转
    int success_count = 0;
    
    for (int i = 0; i < test_frames; i++) {
        auto frame_data = generator.get_next_frame();
        auto tracker_result = tracker.process_frame(frame_data.frame);
        
        if (tracker_result.found) {
            float error = cv::norm(tracker_result.target_center - frame_data.target_position);
            if (error < 10.0f) {
                success_count++;
            }
        }
    }
    
    // 重置生成器
    generator.reset();
    
    return static_cast<float>(success_count) / test_frames;
}

/**
 * @brief 自动调优HSV阈值参数
 */
void auto_tune_hsv_parameters(TrainingFrameGenerator& generator) {
    std::cout << "\n=== Auto-tuning HSV Parameters ===" << std::endl;
    
    // 设置旋转模式
    // float angular_speed = 1.0f;
    TrainingFrameGenerator::AngularVelocityFunction angular_velocity_func = 
        energy_mechanism_velocity_generator();
    generator.set_training_mode(TrainingFrameGenerator::MODE_PENTAGON_ROTATION, 
                                1, 1, angular_velocity_func);
    
    // 测试不同参数组合
    std::vector<float> hue_thresholds = {5.0f, 10.0f, 15.0f, 20.0f, 25.0f, 30.0f};
    std::vector<float> value_thresholds = {15.0f, 25.0f, 35.0f, 45.0f, 55.0f};
    std::vector<float> black_thresholds = {30.0f, 40.0f, 50.0f, 60.0f};
    
    float best_success_rate = 0.0f;
    TrackerConfig best_config;
    
    std::cout << "Testing " << hue_thresholds.size() * value_thresholds.size() * black_thresholds.size() 
              << " parameter combinations..." << std::endl;
    
    int test_count = 0;
    for (float hue_thresh : hue_thresholds) {
        for (float val_thresh : value_thresholds) {
            for (float black_thresh : black_thresholds) {
                test_count++;
                std::cout << "\rTesting combination " << test_count << "..." << std::flush;
                
                TargetTracker tracker;
                TrackerConfig config = tracker.get_config();
                
                // 设置测试参数
                config.hue_similarity_threshold = hue_thresh;
                config.value_min_threshold = val_thresh;
                config.black_value_threshold = black_thresh;
                tracker.set_config(config);
                
                // 测试该参数组合
                float success_rate = test_parameter_config(tracker, generator, 180);
                
                if (success_rate > best_success_rate) {
                    best_success_rate = success_rate;
                    best_config = config;
                    
                    std::cout << "\nNew best! Success rate: " << (best_success_rate * 100.0f) << "%"
                              << " (Hue: " << hue_thresh 
                              << ", Value: " << val_thresh
                              << ", Black: " << black_thresh << ")" << std::endl;
                }
            }
        }
    }
    
    std::cout << "\n\n=== Tuning Results ===" << std::endl;
    std::cout << "Best success rate: " << (best_success_rate * 100.0f) << "%" << std::endl;
    std::cout << "Best parameters:" << std::endl;
    std::cout << "  hue_similarity_threshold: " << best_config.hue_similarity_threshold << std::endl;
    std::cout << "  value_min_threshold: " << best_config.value_min_threshold << std::endl;
    std::cout << "  black_value_threshold: " << best_config.black_value_threshold << std::endl;
    std::cout << "  black_saturation_threshold: " << best_config.black_saturation_threshold << std::endl;
    
    // 保存最佳配置到文件
    std::ofstream config_file("best_parameters.txt");
    if (config_file.is_open()) {
        config_file << "# Best parameters from auto-tuning\n";
        config_file << "hue_similarity_threshold = " << best_config.hue_similarity_threshold << "\n";
        config_file << "value_min_threshold = " << best_config.value_min_threshold << "\n";
        config_file << "black_value_threshold = " << best_config.black_value_threshold << "\n";
        config_file << "black_saturation_threshold = " << best_config.black_saturation_threshold << "\n";
        config_file << "success_rate = " << (best_success_rate * 100.0f) << "%\n";
        config_file.close();
        std::cout << "Best parameters saved to 'best_parameters.txt'" << std::endl;
    }
}

/**
 * @brief 调试特定失败帧
 */
void debug_specific_frame(const std::string& image_path) {
    std::cout << "\n=== Debugging specific frame ===" << std::endl;
    
    cv::Mat img = cv::imread(image_path);
    if (img.empty()) {
        std::cout << "ERROR: Cannot load image: " << image_path << std::endl;
        return;
    }
    
    // 使用不同参数测试同一帧
    std::vector<float> test_thresholds = {5.0f, 10.0f, 15.0f, 20.0f, 25.0f, 30.0f};
    
    for (float threshold : test_thresholds) {
        TargetTracker tracker;
        TrackerConfig config = tracker.get_config();
        config.hue_similarity_threshold = threshold;
        tracker.set_config(config);
        
        auto result = tracker.process_frame(img);
        std::cout << "Hue threshold " << threshold << ": ";
        std::cout << (result.found ? "FOUND" : "NOT FOUND");
        if (result.found) {
            std::cout << " at (" << result.target_center.x << ", " << result.target_center.y << ")";
        }
        std::cout << std::endl;
    }
}

/**
 * @brief 分析颜色相似性问题
 */
void analyze_color_similarity_issue(TrainingFrameGenerator& generator) {
    std::cout << "\n=== Analyzing Color Similarity Issue ===" << std::endl;
    
    // 收集一些关键帧进行分析
    generator.set_training_mode(TrainingFrameGenerator::MODE_PENTAGON_ROTATION, 0.5f);
    
    // 在关键角度采样帧
    std::vector<float> test_angles = {0.0f, 72.0f, 144.0f, 216.0f, 288.0f};
    
    for (float angle : test_angles) {
        // 重置生成器到特定角度
        generator.reset();
        generator.set_training_mode(TrainingFrameGenerator::MODE_PENTAGON_ROTATION, angle * M_PI / 180.0f);
        
        auto frame_data = generator.get_next_frame();
        
        std::cout << "\nAngle " << angle << " degrees:" << std::endl;
        
        // 使用不同参数测试
        TargetTracker tracker1, tracker2;
        
        // 宽松参数
        TrackerConfig config1 = tracker1.get_config();
        config1.hue_similarity_threshold = 30.0f;
        tracker1.set_config(config1);
        
        // 严格参数
        TrackerConfig config2 = tracker2.get_config();
        config2.hue_similarity_threshold = 10.0f;
        tracker2.set_config(config2);
        
        auto result1 = tracker1.process_frame(frame_data.frame);
        auto result2 = tracker2.process_frame(frame_data.frame);
        
        std::cout << "  Loose (30°): " << (result1.found ? "FOUND" : "NOT FOUND");
        if (result1.found) {
            float error = cv::norm(result1.target_center - frame_data.target_position);
            std::cout << " (error: " << error << ")";
        }
        
        std::cout << "\n  Strict (10°): " << (result2.found ? "FOUND" : "NOT FOUND");
        if (result2.found) {
            float error = cv::norm(result2.target_center - frame_data.target_position);
            std::cout << " (error: " << error << ")";
        }
        std::cout << std::endl;
        
        // 保存有问题的帧
        if (result1.found != result2.found) {
            std::string filename = "issue_angle_" + std::to_string((int)angle) + ".jpg";
            cv::imwrite(filename, frame_data.frame);
            std::cout << "  Frame saved to " << filename << std::endl;
        }
    }
}

void auto_adjust_parameters(cv::Mat& test_image) {
    // std::cout << "\n=== 改进的参数分析 ===" << std::endl;
    
    // // 1. 使用饱和度掩膜（已知有效的方法）
    // cv::Mat hsv, saturation_mask;
    // cv::cvtColor(test_image, hsv, cv::COLOR_BGR2HSV);
    // std::vector<cv::Mat> hsv_channels;
    // cv::split(hsv, hsv_channels);
    
    // // 使用与追踪器相同的阈值
    // cv::threshold(hsv_channels[1], saturation_mask, SATURATION_THRESHOLD, 255, cv::THRESH_BINARY);
    // cv::imwrite("debug_for_analysis.png", saturation_mask);
    
    // // 2. 分析轮廓
    // std::vector<std::vector<cv::Point>> contours;
    // cv::findContours(saturation_mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // std::cout << "使用饱和度掩膜找到 " << contours.size() << " 个轮廓" << std::endl;
    
    // if(!contours.empty()) {
    //     std::vector<double> areas;
    //     std::vector<cv::Rect> bounding_boxes;
        
    //     for(size_t i = 0; i < contours.size(); i++) {
    //         double area = cv::contourArea(contours[i]);
    //         areas.push_back(area);
            
    //         cv::Rect rect = cv::boundingRect(contours[i]);
    //         bounding_boxes.push_back(rect);
            
    //         std::cout << "轮廓 " << i << ": 面积=" << area 
    //                  << " 像素, 边界框=" << rect.width << "x" << rect.height << std::endl;
    //     }
        
    //     // 排序并分析
    //     std::sort(areas.begin(), areas.end());
        
    //     std::cout << "\n面积统计:" << std::endl;
    //     std::cout << "  最小值: " << areas.front() << " 像素" << std::endl;
    //     std::cout << "  最大值: " << areas.back() << " 像素" << std::endl;
    //     std::cout << "  中位数: " << areas[areas.size()/2] << " 像素" << std::endl;
        
    //     // 推荐参数
    //     std::cout << "\n推荐追踪器参数:" << std::endl;
    //     int recommended_min = (int)(areas.front() * 0.3); // 比最小值小一些
    //     int recommended_max = (int)(areas.back() * 1.5);  // 比最大值大一些
        
    //     std::cout << "  MIN_BLOB_AREA: " << recommended_min << std::endl;
    //     std::cout << "  MAX_BLOB_AREA: " << recommended_max << std::endl;
        
    //     // 如果之前参数是问题，直接在这里设置
    //     if(recommended_min > 500 || recommended_max < 3000) {
    //         std::cout << "\n⚠️  注意: 之前参数可能不正确!" << std::endl;
    //         std::cout << "  之前: MIN=" << MIN_BLOB_AREA << ", MAX=" << MAX_BLOB_AREA << std::endl;
    //         std::cout << "  建议立即修改为上述推荐值" << std::endl;
    //     }
        
    //     // 计算期望半径（如果找到至少2个轮廓）
    //     if(contours.size() >= 2 && bounding_boxes.size() >= 2) {
    //         cv::Point center1(
    //             bounding_boxes[0].x + bounding_boxes[0].width/2,
    //             bounding_boxes[0].y + bounding_boxes[0].height/2
    //         );
    //         cv::Point center2(
    //             bounding_boxes[1].x + bounding_boxes[1].width/2,
    //             bounding_boxes[1].y + bounding_boxes[1].height/2
    //         );
            
    //         float distance = cv::norm(center1 - center2);
    //         std::cout << "  估算的靶子半径: " << distance << " 像素" << std::endl;
    //         std::cout << "  建议 EXPECTED_RADIUS: " << (int)distance << std::endl;
    //     }
        
    //     // 可视化显示
    //     cv::Mat visual = test_image.clone();
    //     for(size_t i = 0; i < contours.size(); i++) {
    //         cv::drawContours(visual, contours, i, cv::Scalar(0, 255, 0), 2);
            
    //         // 显示面积
    //         cv::Rect rect = bounding_boxes[i];
    //         std::string label = std::to_string((int)areas[i]);
    //         cv::putText(visual, label, 
    //                    cv::Point(rect.x, rect.y - 5),
    //                    cv::FONT_HERSHEY_SIMPLEX, 0.5, 
    //                    cv::Scalar(255, 255, 255), 1);
    //     }
        
    //     cv::imwrite("debug_contours_analysis.png", visual);
    //     cv::imshow("轮廓分析结果", visual);
    //     cv::waitKey(0);
        
    // } else {
    //     std::cout << "错误: 没有找到轮廓!" << std::endl;
    //     std::cout << "可能 SATURATION_THRESHOLD (" << SATURATION_THRESHOLD 
    //               << ") 设置不当" << std::endl;
        
    //     // 显示饱和度通道直方图
    //     cv::Mat sat_channel = hsv_channels[1];
    //     cv::Mat histogram;
    //     int histSize = 256;
    //     float range[] = {0, 256};
    //     const float* histRange = {range};
        
    //     cv::calcHist(&sat_channel, 1, 0, cv::Mat(), histogram, 1, &histSize, &histRange);
        
    //     // 绘制直方图
    //     int hist_w = 512, hist_h = 400;
    //     int bin_w = cvRound((double)hist_w / histSize);
    //     cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(50, 50, 50));
        
    //     cv::normalize(histogram, histogram, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());
        
    //     for(int i = 1; i < histSize; i++) {
    //         cv::line(histImage, 
    //                  cv::Point(bin_w*(i-1), hist_h - cvRound(histogram.at<float>(i-1))),
    //                  cv::Point(bin_w*(i), hist_h - cvRound(histogram.at<float>(i))),
    //                  cv::Scalar(0, 255, 0), 2, 8, 0);
    //     }
        
    //     // 标记当前阈值
    //     int thresh_x = SATURATION_THRESHOLD * hist_w / 256;
    //     cv::line(histImage, cv::Point(thresh_x, 0), cv::Point(thresh_x, hist_h),
    //             cv::Scalar(0, 0, 255), 2);
    //     cv::putText(histImage, "当前阈值", cv::Point(thresh_x + 5, 30),
    //                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
        
    //     cv::imshow("饱和度直方图", histImage);
    //     cv::waitKey(0);
    // }
    TargetTracker tracker;
    auto result = tracker.process_frame(test_image);
    TrackerConfig config;
    while(result.found) {
        float temp = -167.7f;
        if(result.angle - temp < 5.0f && result.angle - temp > -5.0f) {
            std::cout << "Tracker parameters seem OK." << std::endl;
            break;
        } else {
            std::cout << "Tracker parameters may need adjustment." << std::endl;
            config.hue_similarity_threshold +=0.1f;
            std::cout << "Adjusting HUE_COLOR_SIMILARITY_THRESHOLD to " << config.hue_similarity_threshold << std::endl;
            tracker.set_config(config);
            result = tracker.process_frame(test_image);
        }
    }
}



/**
 * @brief 测试圆周运动
 */
void test_circular_motion(TrainingFrameGenerator& generator) {
    std::cout << "\n=== Test 2: Circular Motion ===" << std::endl;
    
    cv::Point2f center(400, 300);
    float radius = 150.0f;
    float angular_speed = 1.0f;
    
    generator.set_circular_motion(center, radius, angular_speed, true);
    
    std::vector<cv::Point2f> trajectory;
    
    for (int i = 0; i < 60; i++) {  // 2秒的圆周运动
        auto frame_data = generator.get_next_frame();
        
        if (i % 10 == 0) {
            std::cout << "Frame " << i 
                      << ": Time=" << frame_data.timestamp << "s"
                      << ", Position=(" << frame_data.target_position.x 
                      << ", " << frame_data.target_position.y << ")"
                      << ", Velocity=(" << frame_data.velocity.x 
                      << ", " << frame_data.velocity.y << ")" << std::endl;
        }
        
        // 记录轨迹
        trajectory.push_back(frame_data.target_position);
        if (trajectory.size() > 100) {
            trajectory.erase(trajectory.begin());
        }
        
        // 显示帧
        cv::Mat display = frame_data.frame.clone();
        
        // 绘制圆周轨迹
        cv::circle(display, center, static_cast<int>(radius), cv::Scalar(200, 200, 200), 1);
        
        // 绘制历史轨迹
        draw_trajectory(display, trajectory, cv::Scalar(255, 150, 0));
        
        // 绘制当前位置
        cv::circle(display, frame_data.target_position, 12, cv::Scalar(0, 255, 255), -1);
        
        // 绘制速度向量
        cv::Point2f vel_end(
            frame_data.target_position.x + frame_data.velocity.x * 5,
            frame_data.target_position.y + frame_data.velocity.y * 5
        );
        cv::arrowedLine(display, frame_data.target_position, vel_end, 
                       cv::Scalar(0, 255, 0), 2);
        
        // 显示信息
        display_info(display, "Circular Motion", 
                    frame_data.target_position, frame_data.velocity,
                    frame_data.timestamp);
        
        cv::imshow("Circular Motion Test", display);
        if (cv::waitKey(33) == 27) break;
    }
    
    cv::destroyAllWindows();
}

/**
 * @brief 测试螺旋线运动
 */
void test_spiral_motion(TrainingFrameGenerator& generator) {
    std::cout << "\n=== Test 3: Spiral Motion ===" << std::endl;
    
    auto spiral_x = [](float t) -> float {
        return 400 + 100 * (1 - std::exp(-0.1 * t)) * std::cos(2 * t);
    };
    
    auto spiral_y = [](float t) -> float {
        return 300 + 100 * (1 - std::exp(-0.1 * t)) * std::sin(2 * t);
    };
    
    generator.set_parametric_motion_mode(spiral_x, spiral_y, 15.0f, false);
    
    std::vector<cv::Point2f> trajectory;
    
    for (int i = 0; i < 180; i++) {  // 6秒的螺旋线运动
        auto frame_data = generator.get_next_frame();
        
        if (i % 30 == 0) {
            std::cout << "Frame " << i 
                      << ": Time=" << frame_data.timestamp << "s"
                      << ", Position=(" << frame_data.target_position.x 
                      << ", " << frame_data.target_position.y << ")" << std::endl;
        }
        
        // 记录轨迹
        trajectory.push_back(frame_data.target_position);
        if (trajectory.size() > 150) {
            trajectory.erase(trajectory.begin());
        }
        
        // 显示帧
        cv::Mat display = frame_data.frame.clone();
        
        // 绘制轨迹
        draw_trajectory(display, trajectory, cv::Scalar(255, 0, 0));
        
        // 绘制当前位置
        cv::circle(display, frame_data.target_position, 10, cv::Scalar(255, 0, 0), -1);
        
        // 显示信息
        display_info(display, "Spiral Motion", 
                    frame_data.target_position, frame_data.velocity,
                    frame_data.timestamp);
        
        cv::imshow("Spiral Motion Test", display);
        if (cv::waitKey(33) == 27) break;
    }
    
    cv::destroyAllWindows();
}

/**
 * @brief 测试正弦波运动
 */
void test_sine_wave_motion(TrainingFrameGenerator& generator) {
    std::cout << "\n=== Test 4: Sine Wave Motion ===" << std::endl;
    
    float amplitude = 100.0f;
    float frequency = 1.0f;
    float speed = 80.0f;
    float direction = 30.0f;
    
    auto fast_sine_x = [start_point = cv::Point2f(100, 300), amplitude, frequency, speed, direction](float t) -> float {
        float dir_rad = direction * M_PI / 180.0f;
        float dx = std::cos(dir_rad);
        float dy = std::sin(dir_rad);
        float perp_dx = -dy;
        
        float main_motion = dx * speed * t;
        float oscillation = perp_dx * amplitude * std::sin(2 * M_PI * frequency * t);
        return start_point.x + main_motion + oscillation;
    };
    
    auto fast_sine_y = [start_point = cv::Point2f(100, 300), amplitude, frequency, speed, direction](float t) -> float {
        float dir_rad = direction * M_PI / 180.0f;
        float dx = std::cos(dir_rad);
        float dy = std::sin(dir_rad);
        float perp_dy = dx;
        
        float main_motion = dy * speed * t;
        float oscillation = perp_dy * amplitude * std::sin(2 * M_PI * frequency * t);
        return start_point.y + main_motion + oscillation;
    };
    
    generator.set_parametric_motion_mode(fast_sine_x, fast_sine_y, 10.0f, true);
    
    std::cout << "Sine wave motion parameters:" << std::endl;
    std::cout << "  Amplitude: " << amplitude << " pixels" << std::endl;
    std::cout << "  Frequency: " << frequency << " Hz" << std::endl;
    std::cout << "  Speed: " << speed << " px/s" << std::endl;
    std::cout << "  Direction: " << direction << " degrees" << std::endl;
    
    std::vector<cv::Point2f> trajectory;
    
    for (int i = 0; i < 300; i++) {  // 10秒的运动
        auto frame_data = generator.get_next_frame();
        
        if (i % 30 == 0) {
            float current_speed = std::sqrt(frame_data.velocity.x * frame_data.velocity.x + 
                                           frame_data.velocity.y * frame_data.velocity.y);
            std::cout << "Frame " << i 
                      << ": Time=" << frame_data.timestamp << "s"
                      << ", Position=(" << frame_data.target_position.x 
                      << ", " << frame_data.target_position.y << ")"
                      << ", Speed=" << current_speed << " px/s" << std::endl;
        }
        
        // 记录轨迹
        trajectory.push_back(frame_data.target_position);
        if (trajectory.size() > 150) {
            trajectory.erase(trajectory.begin());
        }
        
        // 显示帧
        cv::Mat display = frame_data.frame.clone();
        
        // 绘制轨迹
        draw_trajectory(display, trajectory, cv::Scalar(255, 100, 100));
        
        // 绘制当前位置
        cv::circle(display, frame_data.target_position, 10, cv::Scalar(0, 0, 255), -1);
        
        // 绘制速度向量
        cv::Point2f vel_end(
            frame_data.target_position.x + frame_data.velocity.x * 0.5,
            frame_data.target_position.y + frame_data.velocity.y * 0.5
        );
        cv::arrowedLine(display, frame_data.target_position, vel_end,
                       cv::Scalar(0, 255, 0), 2);
        
        // 显示信息
        display_info(display, "Sine Wave Motion", 
                    frame_data.target_position, frame_data.velocity,
                    frame_data.timestamp);
        
        cv::imshow("Sine Wave Motion Test", display);
        if (cv::waitKey(20) == 27) break;
    }
    
    cv::destroyAllWindows();
}

/**
 * @brief 测试线性运动（带反弹）
 */
void test_linear_movement_with_bounce(TrainingFrameGenerator& generator) {
    std::cout << "\n=== Test 5: Linear Movement with Bounce ===" << std::endl;
    
    float vx = 60.0f, vy = 40.0f;
    generator.set_training_mode(TrainingFrameGenerator::MODE_LINEAR_MOVEMENT, vx, vy);
    
    std::cout << "Linear movement with bounce:" << std::endl;
    std::cout << "  Initial velocity: (" << vx << ", " << vy << ") px/s" << std::endl;
    std::cout << "  Image size: " << generator.get_target_sim_center().x * 2 
              << "x" << generator.get_target_sim_center().y * 2 << std::endl;
    
    std::vector<cv::Point2f> trajectory;
    int bounce_count = 0;
    
    for (int i = 0; i < 300; i++) {  // 10秒的运动
        auto frame_data = generator.get_next_frame();
        
        // 检测反弹事件（通过速度变化）
        static cv::Point2f last_velocity(0, 0);
        if (i > 0) {
            float velocity_change = cv::norm(frame_data.velocity - last_velocity);
            if (velocity_change > 10.0f) {  // 速度变化明显，可能是反弹
                bounce_count++;
                std::cout << "[Bounce #" << bounce_count << "] at t=" << frame_data.timestamp << "s" 
                          << ", New velocity: (" << frame_data.velocity.x 
                          << ", " << frame_data.velocity.y << ") px/s" << std::endl;
            }
        }
        last_velocity = frame_data.velocity;
        
        if (i % 30 == 0) {
            std::cout << "Frame " << i 
                      << ": Time=" << frame_data.timestamp << "s"
                      << ", Position=(" << frame_data.target_position.x 
                      << ", " << frame_data.target_position.y << ")" << std::endl;
        }
        
        // 记录轨迹
        trajectory.push_back(frame_data.target_position);
        if (trajectory.size() > 100) {
            trajectory.erase(trajectory.begin());
        }
        
        // 显示帧
        cv::Mat display = frame_data.frame.clone();
        
        // 绘制轨迹
        draw_trajectory(display, trajectory, cv::Scalar(0, 100, 255));
        
        // 绘制当前位置
        cv::circle(display, frame_data.target_position, 12, cv::Scalar(255, 100, 0), -1);
        
        // 绘制速度向量
        cv::Point2f vel_end(
            frame_data.target_position.x + frame_data.velocity.x * 0.3,
            frame_data.target_position.y + frame_data.velocity.y * 0.3
        );
        cv::arrowedLine(display, frame_data.target_position, vel_end,
                       cv::Scalar(0, 255, 0), 2);
        
        // 显示信息
        display_info(display, "Linear Movement with Bounce", 
                    frame_data.target_position, frame_data.velocity,
                    frame_data.timestamp);
        
        // 添加反弹计数
        std::string bounce_text = "Bounces: " + std::to_string(bounce_count);
        cv::putText(display, bounce_text, cv::Point(10, 120), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
        
        cv::imshow("Linear Movement with Bounce", display);
        if (cv::waitKey(33) == 27) break;
    }
    
    cv::destroyAllWindows();
}

/**
 * @brief 测试随机出现模式
 */
void test_random_appearance(TrainingFrameGenerator& generator) {
    std::cout << "\n=== Test 6: Random Appearance Mode ===" << std::endl;
    
    float appearance_interval = 1.5f;
    generator.set_training_mode(TrainingFrameGenerator::MODE_RANDOM_APPEARANCE, appearance_interval);
    
    std::cout << "Random appearance test:" << std::endl;
    std::cout << "  Interval: " << appearance_interval << " seconds" << std::endl;
    std::cout << "  Testing for " << (appearance_interval * 5) << " seconds" << std::endl;
    
    std::vector<cv::Point2f> positions;
    
    for (int i = 0; i < static_cast<int>(appearance_interval * 5 * 30); i++) {
        auto frame_data = generator.get_next_frame();
        
        if (i % 15 == 0) {
            std::cout << "Frame " << i 
                      << ": Time=" << frame_data.timestamp << "s"
                      << ", Position=(" << frame_data.target_position.x 
                      << ", " << frame_data.target_position.y << ")" << std::endl;
        }
        
        // 记录位置
        positions.push_back(frame_data.target_position);
        if (positions.size() > 10) {
            positions.erase(positions.begin());
        }
        
        // 显示帧
        cv::Mat display = frame_data.frame.clone();
        
        // 绘制所有出现过的位置
        for (const auto& pos : positions) {
            cv::circle(display, pos, 8, cv::Scalar(150, 150, 255), -1);
        }
        
        // 绘制当前位置
        cv::circle(display, frame_data.target_position, 15, cv::Scalar(0, 200, 255), 3);
        
        // 显示信息
        display_info(display, "Random Appearance", 
                    frame_data.target_position, frame_data.velocity,
                    frame_data.timestamp);
        
        // 添加位置计数
        std::string pos_text = "Positions: " + std::to_string(positions.size());
        cv::putText(display, pos_text, cv::Point(10, 120), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 100, 200), 2);
        
        cv::imshow("Random Appearance Test", display);
        if (cv::waitKey(33) == 27) break;
    }
    
    cv::destroyAllWindows();
}

/**
 * @brief 测试李萨如图形
 */
void test_lissajous_motion(TrainingFrameGenerator& generator) {
    std::cout << "\n=== Test 7: Lissajous Motion ===" << std::endl;
    
    cv::Point2f center(400, 300);
    float a = 150.0f, b = 100.0f;
    float wx = 2.0f, wy = 3.0f;
    float phase = M_PI / 4;
    
    generator.set_lissajous_motion(center, a, b, wx, wy, phase, true);
    
    std::cout << "Lissajous motion parameters:" << std::endl;
    std::cout << "  Center: (" << center.x << ", " << center.y << ")" << std::endl;
    std::cout << "  Amplitudes: (" << a << ", " << b << ")" << std::endl;
    std::cout << "  Frequencies: (" << wx << ", " << wy << ") rad/s" << std::endl;
    std::cout << "  Phase: " << phase << " rad" << std::endl;
    
    std::vector<cv::Point2f> trajectory;
    
    for (int i = 0; i < 300; i++) {  // 10秒的运动
        auto frame_data = generator.get_next_frame();
        
        if (i % 50 == 0) {
            std::cout << "Frame " << i 
                      << ": Time=" << frame_data.timestamp << "s"
                      << ", Position=(" << frame_data.target_position.x 
                      << ", " << frame_data.target_position.y << ")" << std::endl;
        }
        
        // 记录轨迹
        trajectory.push_back(frame_data.target_position);
        if (trajectory.size() > 200) {
            trajectory.erase(trajectory.begin());
        }
        
        // 显示帧
        cv::Mat display = frame_data.frame.clone();
        
        // 绘制轨迹
        draw_trajectory(display, trajectory, cv::Scalar(200, 0, 200));
        
        // 绘制当前位置
        cv::circle(display, frame_data.target_position, 8, cv::Scalar(200, 0, 200), -1);
        
        // 显示信息
        display_info(display, "Lissajous Motion", 
                    frame_data.target_position, frame_data.velocity,
                    frame_data.timestamp);
        
        // 添加频率信息
        std::string freq_text = "Freq ratio: " + std::to_string(wx) + ":" + std::to_string(wy);
        cv::putText(display, freq_text, cv::Point(10, 120), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(150, 0, 150), 2);
        
        cv::imshow("Lissajous Motion Test", display);
        if (cv::waitKey(33) == 27) break;
    }
    
    cv::destroyAllWindows();
}

// ==================== 辅助函数实现 ====================

/**
 * @brief 在图像上绘制轨迹（安全版本）
 */
void draw_trajectory(cv::Mat& image, const std::vector<cv::Point2f>& trajectory, 
                     const cv::Scalar& color) {
    if (trajectory.size() < 2) return;
    
    for (size_t i = 1; i < trajectory.size(); i++) {
        // 确保点坐标有效
        if (trajectory[i-1].x < 0 || trajectory[i-1].y < 0 || 
            trajectory[i].x < 0 || trajectory[i].y < 0) {
            continue;  // 跳过无效点
        }
        
        // 确保点坐标在图像范围内
        if (trajectory[i-1].x >= image.cols || trajectory[i-1].y >= image.rows ||
            trajectory[i].x >= image.cols || trajectory[i].y >= image.rows) {
            continue;  // 跳过超出图像的点
        }
        
        // 计算线宽（确保至少为1）
        float alpha = static_cast<float>(i) / trajectory.size();
        int line_width = std::max(1, static_cast<int>(3 * alpha));
        
        // 确保线宽不超过最大值
        line_width = std::min(line_width, 10);
        
        try {
            cv::line(image, trajectory[i-1], trajectory[i], 
                    color, line_width);
        } catch (const cv::Exception& e) {
            std::cerr << "Error drawing trajectory line: " << e.what() << std::endl;
            std::cerr << "  Point1: (" << trajectory[i-1].x << ", " << trajectory[i-1].y << ")" << std::endl;
            std::cerr << "  Point2: (" << trajectory[i].x << ", " << trajectory[i].y << ")" << std::endl;
            std::cerr << "  Line width: " << line_width << std::endl;
            // 继续绘制其他线段
        }
    }
}

/**
 * @brief 在图像上显示信息
 */
void display_info(cv::Mat& image, const std::string& title, 
                  const cv::Point2f& position, const cv::Point2f& velocity,
                  float timestamp) {
    // 显示标题
    cv::putText(image, title, cv::Point(10, 30), 
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
    
    // 显示时间
    std::string time_text = "Time: " + std::to_string(timestamp).substr(0, 4) + "s";
    cv::putText(image, time_text, cv::Point(10, 60), 
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);
    
    // 显示位置
    std::string pos_text = "Pos: (" + 
                          std::to_string(static_cast<int>(position.x)) + ", " +
                          std::to_string(static_cast<int>(position.y)) + ")";
    cv::putText(image, pos_text, cv::Point(10, 90), 
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);
    
    // 显示速度
    float speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
    std::string vel_text = "Speed: " + std::to_string(static_cast<int>(speed)) + " px/s";
    cv::putText(image, vel_text, cv::Point(10, 150), 
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 100, 0), 1);
}