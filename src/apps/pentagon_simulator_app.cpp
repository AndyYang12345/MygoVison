#include "TargetSim/PentagonSimulator.hpp"
#include "TargetTracking/TargetTracker.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <cmath>

/**
 * @brief 五角形靶子3D投影模拟器示例程序
 * 
 * 演示如何使用 PentagonSimulator 类来获取3D投影图像，
 * 并应用 TargetTracker 进行目标识别和跟踪
 * 用于仿真训练识别算法
 */

/**
 * @brief 在图像上绘制目标追踪结果
 * @param frame 输入图像
 * @param target_info 追踪结果信息
 * @return 绘制后的图像
 */
cv::Mat draw_tracking_results(cv::Mat frame, const TargetInfo& target_info) {
    if (!target_info.found) {
        // 如果未找到目标
        cv::putText(frame, "No target found", cv::Point(20, frame.rows - 30),
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
        return frame;
    }

    // 绘制标靶板中心
    cv::circle(frame, target_info.board_center, 8, cv::Scalar(255, 255, 0), -1);
    cv::circle(frame, target_info.board_center, 10, cv::Scalar(255, 255, 0), 2);
    cv::putText(frame, "Board", cv::Point(target_info.board_center.x - 20, target_info.board_center.y - 15),
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);

    // 绘制目标色块中心
    cv::circle(frame, target_info.target_center, 6, cv::Scalar(0, 255, 0), -1);
    cv::circle(frame, target_info.target_center, 8, cv::Scalar(0, 255, 0), 2);
    cv::putText(frame, "Target", cv::Point(target_info.target_center.x - 20, target_info.target_center.y - 15),
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

    // 绘制连接线
    cv::line(frame, target_info.board_center, target_info.target_center, 
            cv::Scalar(0, 255, 255), 2);

    // 绘制距离和角度信息
    std::string info = "Distance: " + std::to_string(static_cast<int>(target_info.distance)) + 
                      " px | Angle: " + std::to_string(static_cast<int>(target_info.angle)) + "°";
    cv::putText(frame, info, cv::Point(20, frame.rows - 30),
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

    return frame;
}

int main() {
    std::cout << "\n╔════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║      五角形靶子3D旋转投影模拟器 - 简化版本                ║" << std::endl;
    std::cout << "║  按 'q' 键退出，按 'p' 键暂停/继续                         ║" << std::endl;
    std::cout << "║  交互式调试：W/S 上下旋转  A/D 左右旋转                    ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════════╝\n" << std::endl;

    // 创建配置
    PentagonSimulator::CameraConfig config;
    config.width = 640;
    config.height = 640;
    config.fps = 30.0f;
    config.fx = 381.625f;
    config.fy = 381.625f;
    config.cx = 320.0f;
    config.cy = 320.0f;
    config.position = cv::Point3f(0, -100, 1000);
    config.pitch = 0.35f;
    config.yaw = 0.0f;

    // 创建模拟器
    PentagonSimulator simulator(config);

    std::cout << "配置信息：" << std::endl;
    std::cout << "  分辨率: " << config.width << "x" << config.height << std::endl;
    std::cout << "  焦距: fx=" << config.fx << ", fy=" << config.fy << std::endl;
    std::cout << "  主点: cx=" << config.cx << ", cy=" << config.cy << std::endl;
    std::cout << "  相机位置: (" << config.position.x << ", " << config.position.y << ", " << config.position.z << ")" << std::endl;
    std::cout << "  刷新率: " << config.fps << " FPS\n" << std::endl;

    // 创建目标追踪器
    TargetTracker tracker;
    
    // 配置追踪器（根据需要调整）
    TrackerConfig tracker_config;
    tracker_config.min_blob_area = 100;
    tracker_config.max_blob_area = 5000;
    tracker_config.min_circularity = 0.5f;
    tracker_config.show_debug_windows = false;
    tracker_config.print_debug_info = false;
    tracker.set_config(tracker_config);

    std::cout << "✓ 已创建 TargetTracker" << std::endl;
    std::cout << "✓ TargetTracker 配置：\n"
              << "  最小色块面积: " << tracker_config.min_blob_area << "\n"
              << "  最大色块面积: " << tracker_config.max_blob_area << "\n"
              << "  最小圆形度: " << tracker_config.min_circularity << "\n" << std::endl;

    cv::namedWindow("Pentagon 3D Simulator with Tracking", cv::WINDOW_AUTOSIZE);

    const float rotation_step = 0.02f;  // 旋转步长

    std::cout << "启动模拟器...\n" << std::endl;

    int tracking_success = 0;
    int tracking_total = 0;

    while (true) {
        // 获取当前帧
        cv::Mat frame = simulator.get_frame();

        // 进行目标追踪
        TargetInfo target_info = tracker.process_frame(frame);
        tracking_total++;
        if (target_info.found) {
            tracking_success++;
        }

        // 绘制追踪结果
        cv::Mat display_frame = draw_tracking_results(frame, target_info);

        // 添加信息文字
        std::string info = "Frame: " + std::to_string(simulator.get_frame_count()) +
                          " | Pitch: " + std::to_string(static_cast<int>(simulator.get_camera_pitch() * 180 / M_PI)) +
                          "° Yaw: " + std::to_string(static_cast<int>(simulator.get_camera_yaw() * 180 / M_PI)) + "°" +
                          " | " + (simulator.is_paused() ? "PAUSED" : "PLAYING");

        cv::putText(display_frame, info, cv::Point(20, 40),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);

        // 显示帧
        cv::imshow("Pentagon 3D Simulator with Tracking", display_frame);

        // 获取键盘输入
        int key = cv::waitKey(33);  // ~30 FPS

        if (key == 'q' || key == 27) {  // 'q' 或 ESC
            std::cout << "\n程序退出。" << std::endl;
            break;
        } else if (key == 'p') {  // 'p' 暂停/继续
            if (simulator.is_paused()) {
                simulator.resume();
                std::cout << "继续" << std::endl;
            } else {
                simulator.pause();
                std::cout << "暂停" << std::endl;
            }
        } else if (key == 'w' || key == 'W') {  // W: 向上旋转镜头
            simulator.rotate_camera(-rotation_step, 0);
            std::cout << "镜头向上旋转，俯仰角: " << (simulator.get_camera_pitch() * 180 / M_PI) << "°" << std::endl;
        } else if (key == 's' || key == 'S') {  // S: 向下旋转镜头
            simulator.rotate_camera(rotation_step, 0);
            std::cout << "镜头向下旋转，俯仰角: " << (simulator.get_camera_pitch() * 180 / M_PI) << "°" << std::endl;
        } else if (key == 'a' || key == 'A') {  // A: 镜头向左旋转
            simulator.rotate_camera(0, rotation_step);
            std::cout << "镜头向左旋转，偏航角: " << (simulator.get_camera_yaw() * 180 / M_PI) << "°" << std::endl;
        } else if (key == 'd' || key == 'D') {  // D: 镜头向右旋转
            simulator.rotate_camera(0, -rotation_step);
            std::cout << "镜头向右旋转，偏航角: " << (simulator.get_camera_yaw() * 180 / M_PI) << "°" << std::endl;
        }
    }

    cv::destroyAllWindows();
    
    // 打印统计信息
    std::cout << "\n╔════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                    运行统计信息                            ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════════╝" << std::endl;
    std::cout << "  总帧数: " << tracking_total << std::endl;
    std::cout << "  成功追踪帧数: " << tracking_success << std::endl;
    std::cout << "  追踪成功率: " << (tracking_total > 0 ? (100.0 * tracking_success / tracking_total) : 0.0) << "%" << std::endl;
    tracker.print_statistics();
    
    return 0;
}
