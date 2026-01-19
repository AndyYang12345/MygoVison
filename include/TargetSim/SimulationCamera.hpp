#ifndef SIMULATION_CAMERA_HPP
#define SIMULATION_CAMERA_HPP

#include "TrainingFrameGenerator.hpp"
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>

/**
 * @class SimulationCamera
 * @brief 仿真相机类 - 在3D空间中投影靶平面，模拟真实相机成像过程
 * 
 * 该类提供以下功能：
 * 1. 相机内参数管理（焦距、像素坐标系、畸变等）
 * 2. 靶平面在3D空间中的投影和透视变换
 * 3. 光学效应模拟（焦点、运动模糊等）
 * 4. 为空间仿真提供完整的相机模型
 */
class SimulationCamera {
public:
    /**
     * @struct CameraIntrinsics
     * @brief 相机内参数结构体
     */
    struct CameraIntrinsics {
        // 焦距（像素）
        float fx, fy;
        // 主点（像素坐标）
        float cx, cy;
        // 畸变系数 k1, k2, p1, p2
        float k1, k2, p1, p2;
        // 传感器尺寸（毫米）
        float sensor_width, sensor_height;
        
        CameraIntrinsics() 
            : fx(500.0f), fy(500.0f), cx(400.0f), cy(240.0f),
              k1(0.0f), k2(0.0f), p1(0.0f), p2(0.0f),
              sensor_width(6.0f), sensor_height(4.5f) {}
    };
    
    /**
     * @struct CameraPose
     * @brief 相机位置和姿态
     */
    struct CameraPose {
        // 相机在世界坐标系中的位置（mm）
        cv::Point3f position;
        // 相机的旋转（欧拉角，弧度）
        cv::Point3f rotation;  // (pitch, yaw, roll)
        // 靶平面法向量（默认沿Z轴）
        cv::Point3f target_plane_normal;
        // 靶平面到原点的距离（mm）
        float target_plane_distance;
        
        CameraPose() 
            : position(0, 0, 500),
              rotation(0, 0, 0),
              target_plane_normal(0, 0, 1),
              target_plane_distance(0) {}
    };
    
    /**
     * @struct ProjectionResult
     * @brief 投影结果
     */
    struct ProjectionResult {
        cv::Mat projected_frame;  // 投影后的图像
        std::vector<cv::Point2f> projected_positions;  // 投影后的位置
        std::vector<cv::Point3f> world_positions;      // 世界坐标系中的位置
        bool is_in_view;  // 靶是否在相机视野内
        float distance_to_camera;  // 靶到相机的距离
    };
    
    /**
     * @brief 构造函数
     * @param width 图像宽度（像素）
     * @param height 图像高度（像素）
     * @param fps 帧率（帧/秒）
     */
    explicit SimulationCamera(int width = 800, int height = 600, float fps = 30.0f);
    
    /**
     * @brief 设置相机内参数
     * @param intrinsics 相机内参数
     */
    void set_intrinsics(const CameraIntrinsics& intrinsics);
    
    /**
     * @brief 获取相机内参数
     */
    const CameraIntrinsics& get_intrinsics() const { return _intrinsics; }
    
    /**
     * @brief 设置相机位置和姿态
     * @param pose 相机位置和姿态
     */
    void set_pose(const CameraPose& pose);
    
    /**
     * @brief 获取相机位置和姿态
     */
    const CameraPose& get_pose() const { return _pose; }
    
    /**
     * @brief 通过平移和旋转更新相机位置
     * @param delta_position 位置增量（mm）
     * @param delta_rotation 旋转增量（弧度）
     */
    void update_pose(const cv::Point3f& delta_position, const cv::Point3f& delta_rotation);
    
    /**
     * @brief 设置靶平面参数
     * @param normal 靶平面法向量
     * @param distance 靶平面到原点的距离
     */
    void set_target_plane(const cv::Point3f& normal, float distance);
    
    /**
     * @brief 从仿真流生成器获取训练帧并进行3D投影
     * @param generator 训练帧生成器
     * @return 投影结果
     */
    ProjectionResult project_training_frame(TrainingFrameGenerator& generator);
    
    /**
     * @brief 对指定的2D靶位置进行3D投影
     * @param target_position 目标位置（像素坐标，2D平面）
     * @param original_frame 原始2D帧（用于背景）
     * @return 投影结果
     */
    ProjectionResult project_target_position(const cv::Point2f& target_position,
                                            const cv::Mat& original_frame);
    
    /**
     * @brief 获取透视变换矩阵
     * @param src_points 源点（4个角的坐标）
     * @param dst_points 目标点（4个角的坐标）
     * @return 透视变换矩阵
     */
    cv::Mat get_perspective_matrix(const std::vector<cv::Point2f>& src_points,
                                   const std::vector<cv::Point2f>& dst_points);
    
    /**
     * @brief 计算靶在相机图像平面上的投影位置
     * @param world_position 世界坐标系中的位置（mm）
     * @return 像素坐标
     */
    cv::Point2f world_to_image(const cv::Point3f& world_position);
    
    /**
     * @brief 计算像素坐标对应的世界坐标（基于靶平面）
     * @param image_position 像素坐标
     * @return 世界坐标
     */
    cv::Point3f image_to_world(const cv::Point2f& image_position);
    
    /**
     * @brief 检查点是否在相机视野内
     * @param world_position 世界坐标
     * @return 是否在视野内
     */
    bool is_in_view(const cv::Point3f& world_position);
    
    /**
     * @brief 获取靶平面的四个角在图像平面上的投影
     * @return 四个角的像素坐标（四边形）
     */
    std::vector<cv::Point2f> get_target_plane_projection();
    
    /**
     * @brief 应用运动模糊效果
     * @param frame 输入图像
     * @param velocity 运动速度（像素/帧）
     * @param blur_strength 模糊强度（1-5）
     * @return 添加运动模糊后的图像
     */
    cv::Mat apply_motion_blur(const cv::Mat& frame, const cv::Point2f& velocity, int blur_strength = 2);
    
    /**
     * @brief 应用焦点效果（模拟景深）
     * @param frame 输入图像
     * @param focus_distance 焦距距离（mm）
     * @param aperture 光圈大小（数值越大越模糊）
     * @return 应用焦点效果后的图像
     */
    cv::Mat apply_focus_effect(const cv::Mat& frame, float focus_distance, float aperture = 2.0f);
    
    /**
     * @brief 应用镜头畸变
     * @param frame 输入图像
     * @return 应用畸变后的图像
     */
    cv::Mat apply_lens_distortion(const cv::Mat& frame);
    
    /**
     * @brief 获取旋转矩阵（从欧拉角）
     */
    cv::Mat get_rotation_matrix();
    
    /**
     * @brief 获取相机矩阵（内参矩阵）
     */
    cv::Mat get_camera_matrix() const;
    
    /**
     * @brief 获取图像宽度
     */
    int get_width() const { return _width; }
    
    /**
     * @brief 获取图像高度
     */
    int get_height() const { return _height; }
    
private:
    int _width;
    int _height;
    float _fps;
    
    CameraIntrinsics _intrinsics;
    CameraPose _pose;
    
    // 辅助函数
    cv::Point3f _euler_to_rotation_vector(const cv::Point3f& euler);
    cv::Point3f _rotation_vector_to_euler(const cv::Point3f& rot_vec);
    
    /**
     * @brief 计算目标平面相对于相机的变换
     */
    void _update_plane_transform();
    
    /**
     * @brief 变换矩阵缓存
     */
    cv::Mat _rotation_matrix;
    cv::Mat _camera_matrix;
    bool _matrices_dirty;
};

#endif // SIMULATION_CAMERA_HPP
