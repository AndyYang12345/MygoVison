#include "targetSim/TrainingFrameGenerator.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <functional>

int main() {
    std::cout << "Training Frame Generator Test (Enhanced)" << std::endl;
    std::cout << "==========================================" << std::endl;
    
    // Create generator
    TrainingFrameGenerator generator(800, 600, 30.0f);
    
    std::cout << std::fixed << std::setprecision(2);
    
    // Test 1: Pentagon Rotation Mode
    std::cout << "\n=== Test 1: Pentagon Rotation Mode ===" << std::endl;
    generator.set_training_mode(TrainingFrameGenerator::MODE_PENTAGON_ROTATION, 0.5f);
    
    for (int i = 0; i < 10; i++) {
        auto frame_data = generator.get_next_frame();
        
        std::cout << "Frame " << i 
                  << ": Time=" << frame_data.timestamp << "s"
                  << ", Target Index=" << frame_data.target_index
                  << ", Position=(" << frame_data.target_position.x 
                  << ", " << frame_data.target_position.y << ")" << std::endl;
        
        // Show frame with target marker
        cv::Mat display = frame_data.frame.clone();
        cv::circle(display, frame_data.target_position, 8, cv::Scalar(0, 0, 255), -1);
        cv::putText(display, "Target", 
                    cv::Point(frame_data.target_position.x + 10, frame_data.target_position.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
        
        cv::imshow("Pentagon Rotation Test", display);
        cv::waitKey(100);
    }
    
    cv::destroyAllWindows();
    
    // Test 2: Parametric Motion - Circular Motion
    std::cout << "\n=== Test 2: Circular Motion (Parametric) ===" << std::endl;
    
    // 使用预定义的圆周运动
    cv::Point2f center(400, 300);
    float radius = 150.0f;
    float angular_speed = 1.0f;  // 1 rad/s
    
    generator.set_circular_motion(center, radius, angular_speed, true);
    
    for (int i = 0; i < 60; i++) {  // 2秒的圆周运动
        auto frame_data = generator.get_next_frame();
        
        if (i % 10 == 0) {  // 每10帧输出一次
            std::cout << "Frame " << i 
                      << ": Time=" << frame_data.timestamp << "s"
                      << ", Position=(" << frame_data.target_position.x 
                      << ", " << frame_data.target_position.y << ")"
                      << ", Velocity=(" << frame_data.velocity.x 
                      << ", " << frame_data.velocity.y << ")" << std::endl;
        }
        
        // Show frame with motion visualization
        cv::Mat display = frame_data.frame.clone();
        
        // 绘制圆周轨迹
        cv::circle(display, center, static_cast<int>(radius), cv::Scalar(200, 200, 200), 1);
        
        // 绘制当前位置
        cv::circle(display, frame_data.target_position, 12, cv::Scalar(0, 255, 255), -1);
        
        // 绘制速度向量
        cv::Point2f vel_end(
            frame_data.target_position.x + frame_data.velocity.x * 5,
            frame_data.target_position.y + frame_data.velocity.y * 5
        );
        cv::arrowedLine(display, frame_data.target_position, vel_end, 
                       cv::Scalar(0, 255, 0), 2);
        
        // 添加信息
        std::string info = "Circular Motion - Angular Speed: " + std::to_string(angular_speed) + " rad/s";
        cv::putText(display, info, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
        
        std::string time_text = "Time: " + std::to_string(frame_data.timestamp).substr(0, 4) + "s";
        cv::putText(display, time_text, cv::Point(10, 60), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);
        
        cv::imshow("Circular Motion Test", display);
        if (cv::waitKey(33) == 27) break;  // ~30 FPS
    }
    
    cv::destroyAllWindows();
    
    // Test 3: Parametric Motion - Custom Function
    std::cout << "\n=== Test 3: Custom Parametric Motion ===" << std::endl;
    
    // 自定义参数方程：螺旋线运动
    auto spiral_x = [](float t) -> float {
        return 400 + 100 * (1 - std::exp(-0.1 * t)) * std::cos(2 * t);
    };
    
    auto spiral_y = [](float t) -> float {
        return 300 + 100 * (1 - std::exp(-0.1 * t)) * std::sin(2 * t);
    };
    
    generator.set_parametric_motion_mode(spiral_x, spiral_y, 15.0f, false);
    
    for (int i = 0; i < 180; i++) {  // 6秒的螺旋线运动
        auto frame_data = generator.get_next_frame();
        
        if (i % 30 == 0) {  // 每秒输出一次
            std::cout << "Frame " << i 
                      << ": Time=" << frame_data.timestamp << "s"
                      << ", Position=(" << frame_data.target_position.x 
                      << ", " << frame_data.target_position.y << ")" << std::endl;
        }
        
        // Show frame
        cv::Mat display = frame_data.frame.clone();
        
        // 绘制当前位置
        cv::circle(display, frame_data.target_position, 10, cv::Scalar(255, 0, 0), -1);
        
        // 绘制轨迹点（记录历史位置）
        static std::vector<cv::Point2f> trajectory;
        trajectory.push_back(frame_data.target_position);
        if (trajectory.size() > 100) {
            trajectory.erase(trajectory.begin());
        }
        
        // 绘制轨迹
        for (size_t j = 1; j < trajectory.size(); j++) {
            int alpha = static_cast<int>(255 * j / trajectory.size());
            cv::line(display, trajectory[j-1], trajectory[j], 
                    cv::Scalar(255, 0, 0, alpha), 2);
        }
        
        std::string info = "Spiral Motion - Custom Parametric";
        cv::putText(display, info, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
        
        cv::imshow("Custom Parametric Motion", display);
        if (cv::waitKey(33) == 27) break;
    }
    
    cv::destroyAllWindows();
    
    // Test 4: Sine Wave Motion
   // 在测试文件中，修改sine运动测试部分：
    // Test 4: Sine Wave Motion - 更快的速度和更大的范围
    std::cout << "\n=== Test 4: Sine Wave Motion (Enhanced) ===" << std::endl;

    // 使用更大的振幅和更高的频率
    float amplitude = 100.0f;    // 振幅100像素
    float frequency = 1.0f;      // 频率1Hz
    float speed = 80.0f;         // 前进速度80像素/秒
    float direction = 30.0f;     // 30度方向

    // 创建自定义的快速正弦运动
    auto fast_sine_x = [start_point = cv::Point2f(100, 300), amplitude, frequency, speed, direction](float t) -> float {
        float dir_rad = direction * M_PI / 180.0f;
        float dx = std::cos(dir_rad);
        float dy = std::sin(dir_rad);
        // 垂直方向向量
        float perp_dx = -dy;
        
        float main_motion = dx * speed * t;
        float oscillation = perp_dx * amplitude * std::sin(2 * M_PI * frequency * t);
        return start_point.x + main_motion + oscillation;
    };

    auto fast_sine_y = [start_point = cv::Point2f(100, 300), amplitude, frequency, speed, direction](float t) -> float {
        float dir_rad = direction * M_PI / 180.0f;
        float dx = std::cos(dir_rad);
        float dy = std::sin(dir_rad);
        // 垂直方向向量
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

    for (int i = 0; i < 300; i++) {  // 10秒的运动
        auto frame_data = generator.get_next_frame();
        
        if (i % 30 == 0) {  // 每秒输出一次
            std::cout << "Frame " << i 
                    << ": Time=" << frame_data.timestamp << "s"
                    << ", Position=(" << frame_data.target_position.x 
                    << ", " << frame_data.target_position.y << ")"
                    << ", Speed=" << std::sqrt(frame_data.velocity.x * frame_data.velocity.x + 
                                            frame_data.velocity.y * frame_data.velocity.y) 
                    << " px/s" << std::endl;
        }
        
        cv::Mat display = frame_data.frame.clone();
        
        // 绘制轨迹
        static std::vector<cv::Point2f> trajectory;
        trajectory.push_back(frame_data.target_position);
        if (trajectory.size() > 150) {
            trajectory.erase(trajectory.begin());
        }
        
        // 绘制轨迹线
        for (size_t j = 1; j < trajectory.size(); j++) {
            int alpha = static_cast<int>(200 * j / trajectory.size());
            cv::line(display, trajectory[j-1], trajectory[j], 
                    cv::Scalar(255, 100, 100, alpha), 2);
        }
        
        // 绘制当前点
        cv::circle(display, frame_data.target_position, 10, cv::Scalar(0, 0, 255), -1);
        
        // 绘制速度向量
        cv::Point2f vel_end(
            frame_data.target_position.x + frame_data.velocity.x * 0.5,
            frame_data.target_position.y + frame_data.velocity.y * 0.5
        );
        cv::arrowedLine(display, frame_data.target_position, vel_end,
                    cv::Scalar(0, 255, 0), 2);
        
        // 显示信息
        std::string info = "Fast Sine Wave Motion";
        cv::putText(display, info, cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
        
        std::string speed_text = "Speed: " + 
            std::to_string(static_cast<int>(std::sqrt(frame_data.velocity.x * frame_data.velocity.x + 
                                                    frame_data.velocity.y * frame_data.velocity.y))) + 
            " px/s";
        cv::putText(display, speed_text, cv::Point(10, 60), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 100, 0), 2);
        
        cv::imshow("Fast Sine Wave Motion", display);
        if (cv::waitKey(20) == 27) break;  // 更快的刷新率，约50FPS
    }

    
    cv::destroyAllWindows();
    
    std::cout << "\n=== All tests completed! ===" << std::endl;
    std::cout << "Summary:" << std::endl;
    std::cout << "1. Pentagon Rotation Mode" << std::endl;
    std::cout << "2. Circular Motion (Parametric)" << std::endl;
    std::cout << "3. Custom Parametric Motion (Spiral)" << std::endl;
    std::cout << "4. Sine Wave Motion" << std::endl;
    
    return 0;
}