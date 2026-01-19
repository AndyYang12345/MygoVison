#include "TargetSim/SimulationCamera.hpp"
#include "TargetSim/TrainingFrameGenerator.hpp"
#include <iostream>
#include <opencv2/highgui.hpp>

/**
 * @brief 测试应用：演示仿真相机的3D投影功能
 * 
 * 该程序演示：
 * 1. 创建仿真相机对象
 * 2. 设置相机内参和位置
 * 3. 与仿真流集成
 * 4. 进行3D投影和透视变换
 */
int main() {
    std::cout << "=== SimulationCamera 仿真相机测试 ===" << std::endl;
    
    // 1. 创建仿真相机 (800x600, 30FPS)
    SimulationCamera camera(800, 600, 30.0f);
    std::cout << "✓ 相机已创建" << std::endl;
    
    // 2. 配置相机内参数（标准的相机标定参数）
    SimulationCamera::CameraIntrinsics intrinsics;
    intrinsics.fx = 400.0f;  // X方向焦距
    intrinsics.fy = 400.0f;  // Y方向焦距
    intrinsics.cx = 400.0f;  // 主点X
    intrinsics.cy = 300.0f;  // 主点Y
    intrinsics.k1 = 0.0f;    // 径向畸变系数1
    intrinsics.k2 = 0.0f;    // 径向畸变系数2
    intrinsics.p1 = 0.0f;    // 切向畸变系数1
    intrinsics.p2 = 0.0f;    // 切向畸变系数2
    
    camera.set_intrinsics(intrinsics);
    std::cout << "✓ 相机内参已设置 (fx=" << intrinsics.fx << ", fy=" << intrinsics.fy << ")" << std::endl;
    
    // 3. 配置相机位置和姿态
    SimulationCamera::CameraPose pose;
    pose.position = cv::Point3f(0, 0, 500);   // 相机在Z=500mm处
    pose.rotation = cv::Point3f(0, 0, 0);     // 无旋转
    pose.target_plane_normal = cv::Point3f(0, 0, 1);  // 靶平面法向沿Z轴
    pose.target_plane_distance = 500;  // 靶平面距离
    
    camera.set_pose(pose);
    std::cout << "✓ 相机位置和姿态已设置" << std::endl;
    std::cout << "  - 相机位置: (" << pose.position.x << ", " 
              << pose.position.y << ", " << pose.position.z << ") mm" << std::endl;
    
    // 4. 创建仿真帧生成器
    TrainingFrameGenerator generator(800, 600, 30.0f);
    
    // 设置循环运动模式：圆周运动
    cv::Point2f center(400, 300);
    float radius = 150.0f;
    float angular_speed = 1.0f;  // 弧度/秒
    generator.set_circular_motion(center, radius, angular_speed, true);
    std::cout << "✓ 仿真帧生成器已创建，设置圆周运动模式" << std::endl;
    
    // 5. 生成若干帧进行投影测试
    std::cout << "\n=== 开始生成投影帧 ===" << std::endl;
    
    int num_frames = 10;
    for (int i = 0; i < num_frames; ++i) {
        // 投影训练帧
        auto result = camera.project_training_frame(generator);
        
        std::cout << "\n帧 " << (i + 1) << "/" << num_frames << ":" << std::endl;
        std::cout << "  - 在视野内: " << (result.is_in_view ? "是" : "否") << std::endl;
        
        if (!result.projected_positions.empty()) {
            std::cout << "  - 投影位置: (" << result.projected_positions[0].x << ", " 
                      << result.projected_positions[0].y << ") 像素" << std::endl;
        }
        
        if (!result.world_positions.empty()) {
            std::cout << "  - 世界坐标: (" << result.world_positions[0].x << ", "
                      << result.world_positions[0].y << ", " 
                      << result.world_positions[0].z << ") mm" << std::endl;
        }
        
        std::cout << "  - 距相机距离: " << result.distance_to_camera << " mm" << std::endl;
        
        // 保存前三帧投影图像用于验证
        if (i < 3) {
            std::string filename = "projected_frame_" + std::to_string(i + 1) + ".jpg";
            if (!result.projected_frame.empty()) {
                cv::imwrite(filename, result.projected_frame);
                std::cout << "  - 已保存图像: " << filename << std::endl;
            }
        }
    }
    
    // 6. 测试相机位置更新
    std::cout << "\n=== 测试相机位置更新 ===" << std::endl;
    cv::Point3f delta_position(100, 0, 0);
    cv::Point3f delta_rotation(0, 0.1f, 0);
    camera.update_pose(delta_position, delta_rotation);
    std::cout << "✓ 相机位置已更新（+100mm X, +0.1rad Yaw）" << std::endl;
    
    // 获取更新后的投影
    generator.set_current_time(0);  // 重置时间
    auto result = camera.project_training_frame(generator);
    std::cout << "✓ 更新后的投影距离: " << result.distance_to_camera << " mm" << std::endl;
    
    // 7. 测试光学效果
    std::cout << "\n=== 测试光学效果 ===" << std::endl;
    
    if (!result.projected_frame.empty()) {
        // 测试运动模糊
        cv::Mat motion_blurred = camera.apply_motion_blur(result.projected_frame, 
                                                          cv::Point2f(10, 5), 3);
        cv::imwrite("motion_blur_test.jpg", motion_blurred);
        std::cout << "✓ 运动模糊效果已保存: motion_blur_test.jpg" << std::endl;
        
        // 测试焦点效果
        cv::Mat focused = camera.apply_focus_effect(result.projected_frame, 400, 2.0f);
        cv::imwrite("focus_effect_test.jpg", focused);
        std::cout << "✓ 焦点效果已保存: focus_effect_test.jpg" << std::endl;
    }
    
    // 8. 输出相机参数总结
    std::cout << "\n=== 相机参数总结 ===" << std::endl;
    std::cout << "相机分辨率: " << camera.get_width() << "x" << camera.get_height() << " 像素" << std::endl;
    
    const auto& intrinsics_final = camera.get_intrinsics();
    std::cout << "相机内参:" << std::endl;
    std::cout << "  - 焦距: (" << intrinsics_final.fx << ", " << intrinsics_final.fy << ")" << std::endl;
    std::cout << "  - 主点: (" << intrinsics_final.cx << ", " << intrinsics_final.cy << ")" << std::endl;
    
    const auto& pose_final = camera.get_pose();
    std::cout << "相机位置: (" << pose_final.position.x << ", " 
              << pose_final.position.y << ", " << pose_final.position.z << ") mm" << std::endl;
    std::cout << "相机旋转: (" << pose_final.rotation.x << ", " 
              << pose_final.rotation.y << ", " << pose_final.rotation.z << ") rad" << std::endl;
    
    // 获取靶平面投影
    auto plane_proj = camera.get_target_plane_projection();
    std::cout << "\n靶平面投影 (4个角点，像素坐标):" << std::endl;
    for (size_t i = 0; i < plane_proj.size(); ++i) {
        std::cout << "  角 " << (i + 1) << ": (" << plane_proj[i].x << ", " 
                  << plane_proj[i].y << ")" << std::endl;
    }
    
    std::cout << "\n✅ SimulationCamera 测试完成！" << std::endl;
    std::cout << "后续可以用这个相机类进行复杂的3D空间仿真。" << std::endl;
    
    return 0;
}
