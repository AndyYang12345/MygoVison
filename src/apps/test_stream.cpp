// test_stream_demo.cpp
#include "targetStreamSimulator.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

// 显示帧并处理用户输入
bool show_frame_with_control(const std::string& window_name, 
                            cv::Mat& frame, 
                            float current_time,
                            const std::string& info = "") {
    // 在图像上添加信息
    std::string time_text = "时间: " + std::to_string(current_time).substr(0, 5) + "s";
    cv::putText(frame, time_text, cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
    
    if (!info.empty()) {
        cv::putText(frame, info, cv::Point(10, 60), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);
    }
    
    // 显示控制说明
    cv::putText(frame, "ESC:退出 空格:暂停/继续 R:重置", 
                cv::Point(10, frame.rows - 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    
    cv::imshow(window_name, frame);
    
    int key = cv::waitKey(30); // 约33fps
    if (key == 27) { // ESC键
        return false;
    } else if (key == ' ') { // 空格键暂停/继续
        cv::waitKey(0);
    } else if (key == 'r' || key == 'R') { // R键重置
        // 需要在主函数中处理
    }
    
    return true;
}

// 演示1：基本旋转
void demo_basic_rotation() {
    std::cout << "\n=== 演示1：基本旋转 ===" << std::endl;
    std::cout << "固定位置，匀速旋转，固定颜色" << std::endl;
    
    TargetStreamSimulator stream(800, 600, 30.0f);
    
    // 配置基础参数
    stream.configure_basic(cv::Scalar(0, 0, 255), "pentagon", 2);
    
    // 固定颜色
    std::vector<cv::Scalar> fixed_colors = {
        cv::Scalar(0, 255, 0),     // 绿
        cv::Scalar(255, 0, 0),     // 蓝
        cv::Scalar(255, 255, 0),   // 青
        cv::Scalar(255, 0, 255),   // 紫
        cv::Scalar(0, 255, 255)    // 黄
    };
    
    // 匀速旋转：π/2 rad/s (90度/秒)
    auto rotation_func = TargetStreamSimulator::constant_rotation(M_PI / 2);
    
    // 使用预置的中心位置函数（默认在画面中心）
    stream.configure_fixed_color_motion(fixed_colors, rotation_func);
    
    cv::namedWindow("基本旋转演示", cv::WINDOW_AUTOSIZE);
    
    float time = 0.0f;
    float delta_time = 1.0f / 30.0f;
    
    std::cout << "开始演示，按ESC退出..." << std::endl;
    
    while (true) {
        cv::Mat frame = stream.get_frame_at_time(time);
        
        // 显示当前旋转角度
        float angle_deg = stream.get_current_rotation() * 180 / M_PI;
        std::string info = "旋转角度: " + std::to_string(static_cast<int>(angle_deg)) + "°";
        
        if (!show_frame_with_control("基本旋转演示", frame, time, info)) {
            break;
        }
        
        time += delta_time;
    }
    
    cv::destroyWindow("基本旋转演示");
}

// 演示2：随机颜色+圆形运动
void demo_random_color_circular() {
    std::cout << "\n=== 演示2：随机颜色圆形运动 ===" << std::endl;
    std::cout << "圆形轨迹，匀速旋转，随机颜色(2秒变化一次)" << std::endl;
    
    TargetStreamSimulator stream(800, 600, 30.0f);
    
    stream.configure_basic(cv::Scalar(0, 0, 255), "pentagon", 0);
    
    // 圆形运动：半径150像素，角速度0.8 rad/s
    auto position_func = TargetStreamSimulator::circular_motion(
        cv::Point2f(400, 300), 150.0f, 0.8f);
    
    // 匀速旋转：1 rad/s
    auto rotation_func = TargetStreamSimulator::constant_rotation(1.0f);
    
    // 随机颜色，每2秒变化一次
    stream.configure_random_color_motion(rotation_func, position_func, 2.0f);
    
    cv::namedWindow("圆形运动演示", cv::WINDOW_AUTOSIZE);
    
    float time = 0.0f;
    float delta_time = 1.0f / 30.0f;
    
    std::cout << "开始演示，按ESC退出..." << std::endl;
    
    while (true) {
        cv::Mat frame = stream.get_frame_at_time(time);
        
        // 显示当前位置
        cv::Point2f center = stream.get_current_center();
        std::string info = "位置: (" + 
                          std::to_string(static_cast<int>(center.x)) + ", " +
                          std::to_string(static_cast<int>(center.y)) + ")";
        
        if (!show_frame_with_control("圆形运动演示", frame, time, info)) {
            break;
        }
        
        time += delta_time;
    }
    
    cv::destroyWindow("圆形运动演示");
}

// 演示3：单色块正弦运动
void demo_single_blob_sine() {
    std::cout << "\n=== 演示3：单色块正弦运动 ===" << std::endl;
    std::cout << "单个蓝色色块，正弦轨迹运动" << std::endl;
    
    TargetStreamSimulator stream(800, 600, 30.0f);
    
    // 正弦运动：振幅120像素，频率0.25Hz
    auto position_func = TargetStreamSimulator::sinusoidal_motion(
        cv::Point2f(400, 300), 120.0f, 0.25f);
    
    // 配置单色块运动
    stream.configure_single_blob_motion(
        cv::Scalar(255, 0, 0),  // 蓝色
        position_func,
        45);  // 45像素大小
    
    cv::namedWindow("单色块正弦运动", cv::WINDOW_AUTOSIZE);
    
    float time = 0.0f;
    float delta_time = 1.0f / 30.0f;
    
    std::cout << "开始演示，按ESC退出..." << std::endl;
    
    while (true) {
        cv::Mat frame = stream.get_frame_at_time(time);
        
        // 显示当前位置
        cv::Point2f center = stream.get_current_center();
        std::string info = "单色块位置: (" + 
                          std::to_string(static_cast<int>(center.x)) + ", " +
                          std::to_string(static_cast<int>(center.y)) + ")";
        
        if (!show_frame_with_control("单色块正弦运动", frame, time, info)) {
            break;
        }
        
        time += delta_time;
    }
    
    cv::destroyWindow("单色块正弦运动");
}

// 演示4：线性运动+颜色渐变
void demo_linear_with_color_gradient() {
    std::cout << "\n=== 演示4：线性运动+颜色渐变 ===" << std::endl;
    std::cout << "直线运动，颜色随时间渐变" << std::endl;
    
    TargetStreamSimulator stream(800, 600, 30.0f);
    
    stream.configure_basic(cv::Scalar(0, 0, 255), "pentagon", 1);
    
    // 线性运动：从(100,300)到(700,300)，速度100像素/秒
    auto position_func = TargetStreamSimulator::linear_motion(
        cv::Point2f(100, 300), cv::Point2f(100, 0));
    
    // 振荡旋转
    auto rotation_func = TargetStreamSimulator::oscillating_rotation(
        M_PI/3, 0.5f);
    
    // 自定义颜色函数：颜色随时间渐变
    auto color_func = [](float t) -> std::vector<cv::Scalar> {
        // 根据时间计算颜色值
        float hue = fmod(t * 50.0f, 360.0f); // 色调随时间变化
        
        // 将HSV转换为BGR的简化版本
        std::vector<cv::Scalar> colors;
        for (int i = 0; i < 5; i++) {
            float offset = i * 72.0f; // 每个色块偏移72度
            float current_hue = fmod(hue + offset, 360.0f);
            
            // 简化的HSV到BGR转换
            int b = static_cast<int>(128 + 127 * sin(current_hue * M_PI / 180.0f));
            int g = static_cast<int>(128 + 127 * sin((current_hue + 120) * M_PI / 180.0f));
            int r = static_cast<int>(128 + 127 * sin((current_hue + 240) * M_PI / 180.0f));
            
            // 确保值在0-255范围内
            b = std::max(0, std::min(255, b));
            g = std::max(0, std::min(255, g));
            r = std::max(0, std::min(255, r));
            
            colors.push_back(cv::Scalar(b, g, r));
        }
        
        return colors;
    };
    
    stream.configure_custom_color_motion(color_func, rotation_func, position_func);
    
    cv::namedWindow("线性运动+颜色渐变", cv::WINDOW_AUTOSIZE);
    
    float time = 0.0f;
    float delta_time = 1.0f / 30.0f;
    
    std::cout << "开始演示，按ESC退出..." << std::endl;
    
    while (true) {
        cv::Mat frame = stream.get_frame_at_time(time);
        
        // 显示当前位置
        cv::Point2f center = stream.get_current_center();
        std::string info = "位置: (" + 
                          std::to_string(static_cast<int>(center.x)) + ", " +
                          std::to_string(static_cast<int>(center.y)) + ")";
        
        if (!show_frame_with_control("线性运动+颜色渐变", frame, time, info)) {
            break;
        }
        
        time += delta_time;
        
        // 重置时间，当靶子移动到边界时
        if (center.x > 750) {
            time = 0.0f;
            std::cout << "重置运动..." << std::endl;
        }
    }
    
    cv::destroyWindow("线性运动+颜色渐变");
}

// 演示5：交互式控制（简化修改版）
void demo_interactive_control() {
    std::cout << "\n=== 演示5：交互式控制 ===" << std::endl;
    std::cout << "使用键盘控制运动参数" << std::endl;
    std::cout << "控制说明:" << std::endl;
    std::cout << "  w/s: 控制Y方向速度" << std::endl;
    std::cout << "  a/d: 控制X方向速度" << std::endl;
    std::cout << "  +/-: 增加/减少旋转速度" << std::endl;
    std::cout << "  c: 切换颜色模式(固定/随机)" << std::endl;
    std::cout << "  r: 重置位置" << std::endl;
    std::cout << "  空格: 暂停/继续" << std::endl;
    std::cout << "  ESC: 退出" << std::endl;
    
    TargetStreamSimulator stream(800, 600, 30.0f);
    
    // 初始化参数
    float rotation_speed = 1.0f; // rad/s
    cv::Point2f velocity(0, 0); // 像素/秒
    bool use_random_colors = false;
    
    stream.configure_basic(cv::Scalar(0, 0, 255), "pentagon", 2);
    
    // 固定颜色
    std::vector<cv::Scalar> fixed_colors = {
        cv::Scalar(0, 255, 0),
        cv::Scalar(255, 0, 0),
        cv::Scalar(255, 255, 0),
        cv::Scalar(255, 0, 255),
        cv::Scalar(0, 255, 255)
    };
    
    auto rotation_func = [&rotation_speed](float t) -> float {
        return rotation_speed * t;
    };
    
    auto position_func = [&velocity](float t) -> cv::Point2f {
        cv::Point2f start(400, 300);
        return cv::Point2f(start.x + velocity.x * t,
                          start.y + velocity.y * t);
    };
    
    if (use_random_colors) {
        stream.configure_random_color_motion(rotation_func, position_func, 2.0f);
    } else {
        stream.configure_fixed_color_motion(fixed_colors, rotation_func, position_func);
    }
    
    cv::namedWindow("交互式控制演示", cv::WINDOW_AUTOSIZE);
    
    float time = 0.0f;
    float delta_time = 1.0f / 30.0f;
    
    bool running = true;
    bool paused = false;
    
    while (running) {
        if (!paused) {
            cv::Mat frame = stream.get_frame_at_time(time);
            
            // 显示控制信息
            std::string info = "旋转速度: " + 
                              std::to_string(static_cast<int>(rotation_speed * 180 / M_PI)) + 
                              "°/s | ";
            info += "速度: (" + 
                    std::to_string(static_cast<int>(velocity.x)) + ", " +
                    std::to_string(static_cast<int>(velocity.y)) + ") px/s | ";
            info += "颜色模式: " + std::string(use_random_colors ? "随机" : "固定");
            
            // 显示控制说明
            std::string controls = "W/S:Y速度 A/D:X速度 +/-:旋转 C:颜色 R:重置 空格:暂停 ESC:退出";
            cv::putText(frame, controls, cv::Point(10, frame.rows - 40),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
            
            // 如果暂停，显示提示
            if (paused) {
                cv::putText(frame, "已暂停 - 按空格继续", cv::Point(300, 50),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            }
            
            cv::imshow("交互式控制演示", frame);
        }
        
        // 处理键盘输入
        int key = cv::waitKey(30);
        bool config_changed = false;
        
        if (key == 27) { // ESC
            running = false;
        } else if (key == ' ') { // 空格键暂停/继续
            paused = !paused;
        } else if (key == 'r' || key == 'R') { // R键重置
            time = 0.0f;
            stream.reset();
            std::cout << "已重置" << std::endl;
        } else if (key == 'c' || key == 'C') { // C键切换颜色模式
            use_random_colors = !use_random_colors;
            config_changed = true;
        } else if (key == 'w' || key == 'W') { // W键上移
            velocity.y -= 5.0f;
            config_changed = true;
        } else if (key == 's' || key == 'S') { // S键下移
            velocity.y += 5.0f;
            config_changed = true;
        } else if (key == 'a' || key == 'A') { // A键左移
            velocity.x -= 5.0f;
            config_changed = true;
        } else if (key == 'd' || key == 'D') { // D键右移
            velocity.x += 5.0f;
            config_changed = true;
        } else if (key == '+' || key == '=') { // +键增加旋转
            rotation_speed += 0.2f;
            config_changed = true;
        } else if (key == '-' || key == '_') { // -键减少旋转
            rotation_speed -= 0.2f;
            if (rotation_speed < 0) rotation_speed = 0;
            config_changed = true;
        }
        
        if (config_changed) {
            std::cout << "更新配置: 旋转=" << rotation_speed 
                      << " rad/s, 速度=(" << velocity.x << "," << velocity.y 
                      << ") px/s, 颜色=" << (use_random_colors ? "随机" : "固定") << std::endl;
            
            if (use_random_colors) {
                stream.configure_random_color_motion(rotation_func, position_func, 2.0f);
            } else {
                stream.configure_fixed_color_motion(fixed_colors, rotation_func, position_func);
            }
        }
        
        if (!paused) {
            time += delta_time;
        }
    }
    
    cv::destroyWindow("交互式控制演示");
}
// 主函数：选择演示场景
int main() {
    std::cout << "==========================================" << std::endl;
    std::cout << "    靶子视频流模拟器演示程序" << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << "请选择演示场景：" << std::endl;
    std::cout << "1. 基本旋转" << std::endl;
    std::cout << "2. 随机颜色圆形运动" << std::endl;
    std::cout << "3. 单色块正弦运动" << std::endl;
    std::cout << "4. 线性运动+颜色渐变" << std::endl;
    std::cout << "5. 交互式控制" << std::endl;
    std::cout << "6. 全部演示" << std::endl;
    std::cout << "0. 退出" << std::endl;
    std::cout << "==========================================" << std::endl;
    
    int choice;
    std::cout << "请输入选择 (0-6): ";
    std::cin >> choice;
    
    switch (choice) {
        case 0:
            std::cout << "程序退出" << std::endl;
            return 0;
            
        case 1:
            demo_basic_rotation();
            break;
            
        case 2:
            demo_random_color_circular();
            break;
            
        case 3:
            demo_single_blob_sine();
            break;
            
        case 4:
            demo_linear_with_color_gradient();
            break;
            
        case 5:
            demo_interactive_control();
            break;
            
        case 6:
            std::cout << "\n开始完整演示..." << std::endl;
            demo_basic_rotation();
            demo_random_color_circular();
            demo_single_blob_sine();
            demo_linear_with_color_gradient();
            demo_interactive_control();
            break;
            
        default:
            std::cout << "无效的选择!" << std::endl;
            break;
    }
    
    std::cout << "\n演示程序结束，感谢使用!" << std::endl;
    cv::destroyAllWindows();
    
    return 0;
}