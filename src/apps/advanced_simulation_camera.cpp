#include "TargetSim/SimulationCamera.hpp"
#include "TargetSim/TrainingFrameGenerator.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/highgui.hpp>

/**
 * @brief 高级仿真示例：多相机3D空间仿真
 * 
 * 该程序演示：
 * 1. 多相机同时观察同一目标
 * 2. 相机动态运动模拟
 * 3. 不同视角下的目标追踪
 * 4. 3D空间中的投影坐标统计
 */

/**
 * @brief 创建标准相机配置
 */
SimulationCamera::CameraIntrinsics create_standard_intrinsics(float focal_length = 400.0f) {
    SimulationCamera::CameraIntrinsics intrinsics;
    intrinsics.fx = focal_length;
    intrinsics.fy = focal_length;
    intrinsics.cx = 400.0f;
    intrinsics.cy = 300.0f;
    intrinsics.k1 = 0.0f;
    intrinsics.k2 = 0.0f;
    return intrinsics;
}

/**
 * @brief 创建指定位置的相机
 */
SimulationCamera create_camera_at_position(
    float x, float y, float z,
    float pitch = 0, float yaw = 0, float roll = 0) {
    
    SimulationCamera camera(800, 600, 30.0f);
    
    camera.set_intrinsics(create_standard_intrinsics(400.0f));
    
    SimulationCamera::CameraPose pose;
    pose.position = cv::Point3f(x, y, z);
    pose.rotation = cv::Point3f(pitch, yaw, roll);
    pose.target_plane_distance = 0;
    camera.set_pose(pose);
    
    return camera;
}

/**
 * @brief 多相机仿真场景
 */
class MultiCameraSimulation {
public:
    MultiCameraSimulation()
        : frame_count(0),
          frame_generator(800, 600, 30.0f) {
        
        // 初始化三个相机：正视、左视、俯视
        cameras.push_back(create_camera_at_position(0, 0, 800));      // 正视图
        cameras.push_back(create_camera_at_position(-600, 0, 600));   // 左视图
        cameras.push_back(create_camera_at_position(0, -600, 600));   // 俯视图
        
        // 设置靶的运动：圆周运动
        frame_generator.set_circular_motion(
            cv::Point2f(400, 300),
            150.0f,    // 半径
            1.0f,      // 角速度
            true       // 循环
        );
    }
    
    /**
     * @brief 执行仿真并统计结果
     */
    void run_simulation(int num_frames = 30) {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "多相机3D仿真 - 开始执行" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "相机数量: " << cameras.size() << std::endl;
        std::cout << "总帧数: " << num_frames << std::endl;
        std::cout << std::string(60, '=') << "\n" << std::endl;
        
        std::vector<SimulationCamera::ProjectionResult> frame_results;
        
        for (int frame = 0; frame < num_frames; ++frame) {
            std::cout << "帧 " << (frame + 1) << "/" << num_frames << std::endl;
            
            // 更新相机位置（模拟相机绕目标运动）
            if (frame > 0) {
                update_camera_positions(frame);
            }
            
            // 获取每个相机的投影
            for (size_t cam_idx = 0; cam_idx < cameras.size(); ++cam_idx) {
                auto result = cameras[cam_idx].project_training_frame(frame_generator);
                
                std::cout << "  相机 " << (cam_idx + 1) << ":" << std::endl;
                std::cout << "    在视野内: " << (result.is_in_view ? "✓" : "✗") << std::endl;
                
                if (result.is_in_view && !result.projected_positions.empty()) {
                    cv::Point2f proj_pos = result.projected_positions[0];
                    std::cout << "    投影位置: (" << static_cast<int>(proj_pos.x) << ", "
                              << static_cast<int>(proj_pos.y) << ") px" << std::endl;
                    std::cout << "    距相机: " << static_cast<int>(result.distance_to_camera)
                              << " mm" << std::endl;
                    
                    // 每3帧保存一张多视图图像
                    if (frame % 3 == 0 && cam_idx == 0) {
                        std::string filename = "multiview_frame_" + std::to_string(frame) + "_cam_1.jpg";
                        cv::imwrite(filename, result.projected_frame);
                    }
                }
            }
            std::cout << std::endl;
            
            frame_count++;
        }
        
        print_statistics();
    }
    
private:
    /**
     * @brief 更新相机位置（动态相机运动）
     */
    void update_camera_positions(int frame_num) {
        float t = frame_num * 0.1f;
        
        // 相机1：绕Z轴旋转
        float angle1 = t * 0.1f;
        SimulationCamera::CameraPose pose1;
        pose1.position = cv::Point3f(
            600 * std::cos(angle1),
            0,
            600 + 100 * std::sin(t * 0.2f)
        );
        pose1.rotation = cv::Point3f(0, angle1, 0);
        pose1.target_plane_distance = 0;
        cameras[0].set_pose(pose1);
        
        // 相机2：高度变化
        SimulationCamera::CameraPose pose2;
        pose2.position = cv::Point3f(
            -400,
            300 * std::sin(t * 0.1f),
            500 + 150 * std::cos(t * 0.1f)
        );
        pose2.rotation = cv::Point3f(0.2f, 0.5f, 0);
        pose2.target_plane_distance = 0;
        cameras[1].set_pose(pose2);
        
        // 相机3：绕X轴旋转
        float angle3 = t * 0.15f;
        SimulationCamera::CameraPose pose3;
        pose3.position = cv::Point3f(
            0,
            -500,
            400 + 200 * std::cos(angle3)
        );
        pose3.rotation = cv::Point3f(angle3, 0, 0);
        pose3.target_plane_distance = 0;
        cameras[2].set_pose(pose3);
    }
    
    /**
     * @brief 打印统计信息
     */
    void print_statistics() {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "仿真统计" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "总帧数: " << frame_count << std::endl;
        std::cout << "相机数: " << cameras.size() << std::endl;
        std::cout << "总投影次数: " << (frame_count * cameras.size()) << std::endl;
        
        // 输出最后一个相机的参数
        if (!cameras.empty()) {
            const auto& last_camera = cameras.back();
            std::cout << "\n最后一个相机的参数:" << std::endl;
            std::cout << "  分辨率: " << last_camera.get_width() << "x" 
                      << last_camera.get_height() << std::endl;
            
            const auto& intrinsics = last_camera.get_intrinsics();
            std::cout << "  焦距: " << intrinsics.fx << ", " << intrinsics.fy << std::endl;
            
            const auto& pose = last_camera.get_pose();
            std::cout << "  位置: (" << static_cast<int>(pose.position.x) << ", "
                      << static_cast<int>(pose.position.y) << ", "
                      << static_cast<int>(pose.position.z) << ") mm" << std::endl;
        }
        
        std::cout << std::string(60, '=') << "\n" << std::endl;
    }
    
    std::vector<SimulationCamera> cameras;
    int frame_count;
    TrainingFrameGenerator frame_generator;
};

/**
 * @brief 相机标定仿真
 */
class CalibrationSimulation {
public:
    /**
     * @brief 执行相机标定测试
     */
    static void run_calibration_test() {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "相机标定仿真" << std::endl;
        std::cout << std::string(60, '=') << "\n" << std::endl;
        
        // 创建测试相机
        SimulationCamera camera = create_camera_at_position(0, 0, 600);
        
        // 定义已知的标定点（世界坐标）
        std::vector<cv::Point3f> calibration_points = {
            cv::Point3f(-200, -150, 0),
            cv::Point3f(200, -150, 0),
            cv::Point3f(0, 150, 0),
            cv::Point3f(-200, 0, 0),
            cv::Point3f(200, 0, 0),
            cv::Point3f(0, -150, 0),
        };
        
        std::cout << "标定点数: " << calibration_points.size() << std::endl;
        std::cout << "\n投影验证:\n" << std::endl;
        
        float max_error = 0;
        float total_error = 0;
        int count = 0;
        
        for (const auto& world_pt : calibration_points) {
            // 正向投影
            cv::Point2f image_pt = camera.world_to_image(world_pt);
            
            // 反向投影
            cv::Point3f world_back = camera.image_to_world(image_pt);
            
            // 计算误差（在平面内）
            float error = std::sqrt(
                (world_pt.x - world_back.x) * (world_pt.x - world_back.x) +
                (world_pt.y - world_back.y) * (world_pt.y - world_back.y)
            );
            
            std::cout << "点 " << (count + 1) << ":" << std::endl;
            std::cout << "  原始: (" << static_cast<int>(world_pt.x) << ", "
                      << static_cast<int>(world_pt.y) << ", "
                      << static_cast<int>(world_pt.z) << ")" << std::endl;
            std::cout << "  投影: (" << static_cast<int>(image_pt.x) << ", "
                      << static_cast<int>(image_pt.y) << ")" << std::endl;
            std::cout << "  反投: (" << static_cast<int>(world_back.x) << ", "
                      << static_cast<int>(world_back.y) << ", "
                      << static_cast<int>(world_back.z) << ")" << std::endl;
            std::cout << "  误差: " << error << " mm" << std::endl << std::endl;
            
            max_error = std::max(max_error, error);
            total_error += error;
            count++;
        }
        
        std::cout << "统计信息:" << std::endl;
        std::cout << "  最大误差: " << max_error << " mm" << std::endl;
        std::cout << "  平均误差: " << (total_error / count) << " mm" << std::endl;
        std::cout << std::string(60, '=') << "\n" << std::endl;
    }
};

int main() {
    std::cout << std::string(60, '*') << std::endl;
    std::cout << "*" << std::string(58, ' ') << "*" << std::endl;
    std::cout << "*   SimulationCamera - 高级仿真示例" << std::string(23, ' ') << "*" << std::endl;
    std::cout << "*" << std::string(58, ' ') << "*" << std::endl;
    std::cout << std::string(60, '*') << std::endl;
    
    // 1. 多相机仿真
    MultiCameraSimulation multi_cam_sim;
    multi_cam_sim.run_simulation(20);
    
    // 2. 相机标定仿真
    CalibrationSimulation::run_calibration_test();
    
    std::cout << "✅ 高级仿真完成！\n" << std::endl;
    std::cout << "生成的文件:" << std::endl;
    std::cout << "  - multiview_frame_*.jpg: 多视图投影图像" << std::endl;
    std::cout << "\n这些演示为后续的复杂3D仿真奠定了基础。" << std::endl;
    
    return 0;
}
