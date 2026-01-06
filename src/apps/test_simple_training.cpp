#include "targetSim/TrainingFrameGenerator.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <functional>
#include <vector>

// ==================== 测试函数声明 ====================
void test_pentagon_rotation(TrainingFrameGenerator& generator);
void test_circular_motion(TrainingFrameGenerator& generator);
void test_spiral_motion(TrainingFrameGenerator& generator);
void test_sine_wave_motion(TrainingFrameGenerator& generator);
void test_linear_movement_with_bounce(TrainingFrameGenerator& generator);
void test_random_appearance(TrainingFrameGenerator& generator);
void test_lissajous_motion(TrainingFrameGenerator& generator);

// ==================== 辅助函数 ====================
void draw_trajectory(cv::Mat& image, const std::vector<cv::Point2f>& trajectory, 
                     const cv::Scalar& color);
void display_info(cv::Mat& image, const std::string& title, 
                  const cv::Point2f& position, const cv::Point2f& velocity,
                  float timestamp);

// ==================== 主函数 ====================
int main() {
    std::cout << "Training Frame Generator Test Suite" << std::endl;
    std::cout << "===================================" << std::endl;
    
    // 创建 TrainingFrameGenerator
    TrainingFrameGenerator generator(800, 600, 60.0f);
    std::cout << std::fixed << std::setprecision(2);
    
    // 运行测试
    test_pentagon_rotation(generator);
    // test_circular_motion(generator);
    // test_spiral_motion(generator);
    // test_sine_wave_motion(generator);
    // test_linear_movement_with_bounce(generator);
    // test_random_appearance(generator);
    // test_lissajous_motion(generator);
    
    std::cout << "\n=== All tests completed! ===" << std::endl;
    
    return 0;
}

// ==================== 测试函数实现 ====================

/**
 * @brief 测试随机五角星旋转模式
 */
void test_pentagon_rotation(TrainingFrameGenerator& generator) {
    std::cout << "\n=== Pentagon Rotation Test ===" << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  ESC: Exit" << std::endl;
    std::cout << "  SPACE: Pause/Resume rotation" << std::endl;
    std::cout << "  R: Generate new random pentagon (random position)" << std::endl;
    std::cout << "  C: Generate new pentagon at center" << std::endl;
    std::cout << "  T: Toggle target markers (center, target, line)" << std::endl;
    std::cout << "  +, -: Adjust rotation speed" << std::endl;
    std::cout << "  0: Reset angle to 0°" << std::endl;
    
    // 设置训练模式为五角星旋转
    float angular_speed = 0.5f;
    generator.set_training_mode(TrainingFrameGenerator::MODE_PENTAGON_ROTATION, angular_speed);
    
    // 状态变量
    bool show_markers = true;  // 默认显示标记
    
    // 帧率计算
    auto last_time = std::chrono::high_resolution_clock::now();
    int frame_count = 0;
    float fps = 30.0f;
    int total_frames = 0;
    
    cv::namedWindow("Pentagon Rotation", cv::WINDOW_AUTOSIZE);
    
    std::cout << "\nTest started. Pentagon is auto-rotating at center." << std::endl;
    std::cout << "Target markers are ON (default). Press T to toggle." << std::endl;
    std::cout << "Press SPACE to pause/resume rotation." << std::endl;
    
    while (true) {
        // 计算帧率
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration<float>(current_time - last_time).count();
        frame_count++;
        
        if (elapsed >= 1.0f) {
            fps = frame_count / elapsed;
            frame_count = 0;
            last_time = current_time;
        }
        
        // 获取训练帧 - 让 TrainingFrameGenerator 自己处理暂停
        auto frame_data = generator.get_next_frame();
        
        total_frames++;
        
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
            
            // 6. 在连线中间显示角度信息
            cv::Point2f mid_point(
                (center.x + frame_data.target_position.x) / 2,
                (center.y + frame_data.target_position.y) / 2
            );
            std::string angle_text = std::to_string((int)(generator.get_current_time() * angular_speed * 180 / M_PI)) + "degrees";
            cv::putText(display, angle_text, 
                       cv::Point(mid_point.x - 10, mid_point.y - 10),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }
        
        // // === 在左上角显示控制信息 ===
        // cv::rectangle(display, cv::Point(5, 5), cv::Point(400, 200), cv::Scalar(255, 255, 255, 220), -1);  // 增加高度到200
        // cv::rectangle(display, cv::Point(5, 5), cv::Point(400, 200), cv::Scalar(0, 0, 0), 1);
        
        // // 标题
        // cv::putText(display, "Pentagon Rotation Test", 
        //            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, 
        //            cv::Scalar(0, 0, 0), 2);
        
        // 帧率信息
        cv::putText(display, "FPS: " + std::to_string(int(fps)), 
                   cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                   cv::Scalar(0, 0, 0), 1);
        
        // cv::putText(display, "Frame: " + std::to_string(total_frames), 
        //            cv::Point(10, 85), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
        //            cv::Scalar(0, 0, 0), 1);
        
        // // 时间信息
        // cv::putText(display, "Time: " + std::to_string(frame_data.timestamp).substr(0,4) + "s", 
        //            cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
        //            cv::Scalar(0, 0, 0), 1);
        
        // // 旋转状态
        // std::string rotate_status = "Rotation: ";
        // rotate_status += generator.is_paused() ? "PAUSED" : "RUNNING";
        // cv::Scalar status_color = generator.is_paused() ? cv::Scalar(200, 0, 0) : cv::Scalar(0, 150, 0);
        // cv::putText(display, rotate_status, 
        //            cv::Point(10, 135), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
        //            status_color, 1);
        
        // // 标记状态
        // std::string marker_status = "Markers: ";
        // marker_status += show_markers ? "ON" : "OFF";
        // cv::Scalar marker_color = show_markers ? cv::Scalar(0, 150, 0) : cv::Scalar(200, 0, 0);
        // cv::putText(display, marker_status, 
        //            cv::Point(10, 160), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
        //            marker_color, 1);
        
        // // 当标记关闭时，在信息框中添加一个更明显的提示
        // if (!show_markers) {
        //     cv::putText(display, "! Press T to show markers !", 
        //                cv::Point(10, 185), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
        //                cv::Scalar(200, 0, 0), 1);
        // }
        
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
        } else if (key == 'r' || key == 'R') { // R键生成随机位置的新靶子
            generator.regenerate_pentagon(cv::Point2f(-1, -1)); // 随机位置
            std::cout << "Generated new pentagon at random position" << std::endl;
        } else if (key == 'c' || key == 'C') { // C键生成中心位置的新靶子
            generator.regenerate_pentagon(generator.get_target_sim_center());
            std::cout << "Generated new pentagon at center" << std::endl;
        } else if (key == '+') { // +键增加旋转速度
            angular_speed += 0.1f;
            generator.set_training_mode(TrainingFrameGenerator::MODE_PENTAGON_ROTATION, angular_speed);
            std::cout << "Rotation speed increased to: " << angular_speed << " rad/s" << std::endl;
        } else if (key == '-') { // -键减少旋转速度
            angular_speed = std::max(0.1f, angular_speed - 0.1f);
            generator.set_training_mode(TrainingFrameGenerator::MODE_PENTAGON_ROTATION, angular_speed);
            std::cout << "Rotation speed decreased to: " << angular_speed << " rad/s" << std::endl;
        } else if (key == '0') { // 0键重置角度和时间
            generator.reset();
            std::cout << "Training reset (time = 0, angle = 0)" << std::endl;
        }
        
        // // 每100帧输出一次状态
        // if (total_frames % 100 == 0) {
        //     std::cout << "Frame " << total_frames 
        //               << ", Time: " << frame_data.timestamp << "s"
        //               << ", FPS: " << int(fps) 
        //               << ", Speed: " << angular_speed << " rad/s"
        //               << ", State: " << (generator.is_paused() ? "PAUSED" : "RUNNING")
        //               << ", Markers: " << (show_markers ? "ON" : "OFF") << std::endl;
        // }
    }
    
    cv::destroyAllWindows();
    std::cout << "\nTest completed. Total frames: " << total_frames << std::endl;
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