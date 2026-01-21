#include "TargetSim/SimulationCamera.hpp"
#include "TargetSim/TrainingFrameGenerator.hpp"
#include "TargetSim/TargetSim.hpp"
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <random>

/**
 * @brief 五角形旋转靶子3D投影演示
 * 
 * 功能：
 * 1. 靶子中心固定在空间原点 (0, 0, 0)，面朝 X 轴正方向
 * 2. 相机位置：前方 1m，视角向下约 20°
 * 3. TrainingFrameGenerator 自动生成旋转的靶子帧
 * 4. 将每一帧投影到3D空间，从目标相机观察
 * 5. 使用 OpenCV imshow 实时显示投影结果
 * 6. 支持变速旋转（能量机制）
 */

/**
 * @brief 能量机制变速旋转函数生成器
 * 
 * 生成一个角速度函数：spd(t) = a * sin(ω * t) + b
 * 参数范围：a ∈ [0.780, 1.045], ω ∈ [1.884, 2.000]
 */
TrainingFrameGenerator::AngularVelocityFunction energy_mechanism_velocity_generator() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    
    // 参数范围
    std::uniform_real_distribution<float> a_dist(0.780f, 1.045f);  // a ∈ [0.780, 1.045]
    std::uniform_real_distribution<float> omega_dist(1.884f, 2.000f); // ω ∈ [1.884, 2.000]
    
    // 随机生成参数
    float a = a_dist(gen);
    float omega = omega_dist(gen);
    float b = 2.090f - a;  // b = 2.090 - a
    
    std::cout << "[能量机制] 生成变速参数: "
              << "a = " << a << ", ω = " << omega 
              << ", b = " << b << ", spd(t) = " << a << " * sin(" << omega << " * t) + " << b 
              << std::endl;
    
    return [a, omega, b](float t) -> float {
        return a * std::sin(omega * t) + b;
    };
}

/**
 * @brief 使用透视变换矩阵投影整个图像
 * 
 * 计算源图像四个角在目标相机中的投影，构建透视变换矩阵
 */
cv::Mat reproject_image_3d(const cv::Mat& src_image,
                           SimulationCamera& src_camera,
                           SimulationCamera& dst_camera) {
    int width = src_image.cols;
    int height = src_image.rows;
    
    auto src_intrinsics = src_camera.get_intrinsics();
    auto src_pose = src_camera.get_pose();
    
    // 源图像四个角的坐标
    std::vector<cv::Point2f> src_corners = {
        cv::Point2f(0, 0),
        cv::Point2f(width - 1, 0),
        cv::Point2f(width - 1, height - 1),
        cv::Point2f(0, height - 1)
    };
    
    // 计算这四个角在目标相机中的投影
    std::vector<cv::Point2f> dst_corners;
    
    for (const auto& pt : src_corners) {
        // 反投影到3D世界坐标
        float x_cam = (pt.x - src_intrinsics.cx) / src_intrinsics.fx;
        float y_cam = (pt.y - src_intrinsics.cy) / src_intrinsics.fy;
        
        // 靶子在 Z=0 平面
        float t = src_pose.position.z;
        cv::Point3f world_pt(x_cam * t, y_cam * t, 0.0f);
        
        // 投影到目标相机
        cv::Point2f proj = dst_camera.world_to_image(world_pt);
        dst_corners.push_back(proj);
    }
    
    // 计算透视变换矩阵
    cv::Mat M = cv::getPerspectiveTransform(src_corners, dst_corners);
    
    // 应用透视变换
    cv::Mat result;
    cv::warpPerspective(src_image, result, M, cv::Size(width, height), 
                        cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
    
    return result;
}

int main() {
    std::cout << "\n╔════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║      五角形靶子3D旋转投影实时显示演示（变速旋转）          ║" << std::endl;
    std::cout << "║  按 'q' 键退出，按 'p' 键暂停/继续                         ║" << std::endl;
    std::cout << "║  交互式调试：W/S 上下旋转  A/D 左右旋转                    ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════════╝\n" << std::endl;
    
    // 创建靶子生成器 - 自动生成旋转的靶子（使用能量机制变速旋转）
    TrainingFrameGenerator generator(640, 640, 30.0f);
    auto velocity_func = energy_mechanism_velocity_generator();
    generator.set_training_mode(TrainingFrameGenerator::MODE_PENTAGON_ROTATION, 1.0f, 1.0f, velocity_func);
    
    // 源相机（生成靶子的相机 - 俯视）
    SimulationCamera src_camera(640, 640, 30.0f);
    SimulationCamera::CameraIntrinsics intrinsics;
    intrinsics.fx = 381.625f;
    intrinsics.fy = 381.625f;
    intrinsics.cx = 320.0f;
    intrinsics.cy = 320.0f;
    src_camera.set_intrinsics(intrinsics);
    
    SimulationCamera::CameraPose src_pose;
    src_pose.position = cv::Point3f(0, 0, 800);  // 俯视位置
    src_pose.rotation = cv::Point3f(0, 0, 0);
    src_pose.target_plane_distance = 0;
    src_camera.set_pose(src_pose);
    
    // 观察相机（靶子前方1m，视角向下）
    SimulationCamera dst_camera(640, 640, 30.0f);
    dst_camera.set_intrinsics(intrinsics);
    
    // 相机交互参数
    float camera_pitch = 0.35f;  // 俯仰角（绕X轴）
    float camera_yaw = 0.0f;     // 偏航角（绕Y轴）
    const float rotation_step = 0.02f;  // 旋转步长
    
    SimulationCamera::CameraPose dst_pose;
    dst_pose.position = cv::Point3f(0, -100, 1000);  // 前方1m
    dst_pose.target_plane_distance = 0;
    
    std::cout << "配置信息：" << std::endl;
    std::cout << "  靶子位置: 原点 (0, 0, 0)，面朝 X 轴正方向（围绕自身中心旋转）" << std::endl;
    std::cout << "  相机距离: 1m" << std::endl;
    std::cout << "  相机角度: 向下 ~20°" << std::endl;
    std::cout << "  刷新率: 30 FPS\n" << std::endl;
    
    cv::namedWindow("Rotating Pentagon Target - 3D Projection", cv::WINDOW_AUTOSIZE);
    
    bool paused = false;
    int frame_count = 0;
    
    std::cout << "启动实时显示...\n" << std::endl;
    
    while (true) {
        // 获取靶子图像（已由TrainingFrameGenerator进行旋转处理）
        auto training_frame = generator.get_next_frame(frame_count / 30.0f);
        
        // 更新相机姿态
        dst_pose.rotation = cv::Point3f(camera_pitch, camera_yaw, 0);
        dst_camera.set_pose(dst_pose);
        
        // 直接投影图像
        cv::Mat projected = reproject_image_3d(training_frame.frame, src_camera, dst_camera);
        
        // 添加信息文字
        std::string info = "Frame: " + std::to_string(frame_count) + 
                          " | Pitch: " + std::to_string(static_cast<int>(camera_pitch * 180 / M_PI)) + 
                          "° Yaw: " + std::to_string(static_cast<int>(camera_yaw * 180 / M_PI)) + "°" +
                          " | " + (paused ? "PAUSED" : "PLAYING");
        
        cv::putText(projected, info, cv::Point(20, 40),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
        
        // 显示
        cv::imshow("Rotating Pentagon Target - 3D Projection", projected);
        
        // 获取键盘输入
        int key = cv::waitKey(30);  // ~30 FPS
        
        if (key == 'q' || key == 27) {  // 'q' 或 ESC
            std::cout << "\n程序退出。" << std::endl;
            break;
        } else if (key == 'p') {  // 'p' 暂停/继续
            paused = !paused;
            if (paused) {
                generator.pause();
            } else {
                generator.resume();
            }
            std::cout << (paused ? "暂停" : "继续") << std::endl;
        } else if (key == 'w' || key == 'W') {  // W: 向上旋转镜头（减小俯仰角）
            camera_pitch -= rotation_step;
            std::cout << "镜头向上旋转，俯仰角: " << (camera_pitch * 180 / M_PI) << "°" << std::endl;
        } else if (key == 's' || key == 'S') {  // S: 向下旋转镜头（增加俯仰角）
            camera_pitch += rotation_step;
            std::cout << "镜头向下旋转，俯仰角: " << (camera_pitch * 180 / M_PI) << "°" << std::endl;
        } else if (key == 'a' || key == 'A') {  // A: 镜头向右旋转（增加偏航角）
            camera_yaw += rotation_step;
            std::cout << "镜头向左旋转，偏航角: " << (camera_yaw * 180 / M_PI) << "°" << std::endl;
        } else if (key == 'd' || key == 'D') {  // D: 镜头向左旋转（减小偏航角）
            camera_yaw -= rotation_step;
            std::cout << "镜头向右旋转，偏航角: " << (camera_yaw * 180 / M_PI) << "°" << std::endl;
        }
        
        // 更新帧计数
        if (!paused) {
            frame_count++;
        }
    }
    
    cv::destroyAllWindows();
    return 0;
}
