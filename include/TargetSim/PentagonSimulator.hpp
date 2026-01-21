#pragma once

#include "SimulationCamera.hpp"
#include "TrainingFrameGenerator.hpp"
#include <opencv2/core.hpp>
#include <memory>

/**
 * @brief 五角形靶子3D旋转投影模拟器
 * 
 * 将靶子生成和3D投影功能封装成类，提供简洁的接口用于仿真训练。
 * 支持：
 * - 灵活配置相机参数和分辨率
 * - 获取实时3D投影图像
 * - 调整相机视角（俯仰角、偏航角）
 * - 控制旋转动画（暂停、继续、重置）
 */
class PentagonSimulator {
public:
    /**
     * @brief 相机配置参数结构
     */
    struct CameraConfig {
        int width;
        int height;
        float fps;
        float fx;
        float fy;
        float cx;
        float cy;
        cv::Point3f position;      // 相机位置
        float pitch;               // 俯仰角（绕X轴）
        float yaw;                 // 偏航角（绕Y轴）
        
        // 默认构造函数
        CameraConfig(int w = 640, int h = 640, float f = 30.0f)
            : width(w), height(h), fps(f),
              fx(381.625f), fy(381.625f), cx(320.0f), cy(320.0f),
              position(0, -100, 1000), pitch(0.35f), yaw(0.0f) {}
    };

    /**
     * @brief 构造函数，使用默认配置
     */
    PentagonSimulator();

    /**
     * @brief 构造函数，使用自定义配置
     * @param config 相机配置参数
     */
    explicit PentagonSimulator(const CameraConfig& config);

    /**
     * @brief 析构函数
     */
    ~PentagonSimulator();

    /**
     * @brief 初始化模拟器（可选，已在构造函数中调用）
     * @param config 相机配置参数
     */
    void initialize(const CameraConfig& config);

    /**
     * @brief 获取下一帧3D投影图像
     * @return 3D投影后的图像（cv::Mat）
     */
    cv::Mat get_frame();

    /**
     * @brief 获取当前帧编号
     * @return 帧计数
     */
    int get_frame_count() const { return frame_count_; }

    /**
     * @brief 获取当前时间戳（秒）
     * @return 时间戳
     */
    float get_timestamp() const { return frame_count_ / 30.0f; }

    /**
     * @brief 设置相机俯仰角（绕X轴）
     * @param pitch 俯仰角（弧度）
     */
    void set_camera_pitch(float pitch);

    /**
     * @brief 设置相机偏航角（绕Y轴）
     * @param yaw 偏航角（弧度）
     */
    void set_camera_yaw(float yaw);

    /**
     * @brief 获取当前俯仰角
     * @return 俯仰角（弧度）
     */
    float get_camera_pitch() const { return camera_pitch_; }

    /**
     * @brief 获取当前偏航角
     * @return 偏航角（弧度）
     */
    float get_camera_yaw() const { return camera_yaw_; }

    /**
     * @brief 旋转相机（相对调整）
     * @param delta_pitch 俯仰角增量（弧度）
     * @param delta_yaw 偏航角增量（弧度）
     */
    void rotate_camera(float delta_pitch, float delta_yaw);

    /**
     * @brief 暂停旋转动画
     */
    void pause();

    /**
     * @brief 继续旋转动画
     */
    void resume();

    /**
     * @brief 重置到初始状态
     */
    void reset();

    /**
     * @brief 检查是否暂停
     * @return true 表示暂停中，false 表示运行中
     */
    bool is_paused() const { return paused_; }

    /**
     * @brief 获取当前相机配置
     * @return 相机配置
     */
    CameraConfig get_config() const { return config_; }

private:
    /**
     * @brief 能量机制变速旋转函数生成器
     * @return 角速度函数
     */
    TrainingFrameGenerator::AngularVelocityFunction energy_mechanism_velocity_generator();

    /**
     * @brief 使用透视变换矩阵投影整个图像
     * @param src_image 源图像
     * @param src_camera 源相机
     * @param dst_camera 目标相机
     * @return 投影后的图像
     */
    cv::Mat reproject_image_3d(const cv::Mat& src_image,
                               SimulationCamera& src_camera,
                               SimulationCamera& dst_camera);

    /**
     * @brief 更新相机姿态
     */
    void update_camera_pose();

    // 成员变量
    CameraConfig config_;
    std::unique_ptr<TrainingFrameGenerator> generator_;
    std::unique_ptr<SimulationCamera> src_camera_;
    std::unique_ptr<SimulationCamera> dst_camera_;
    
    float camera_pitch_;
    float camera_yaw_;
    bool paused_;
    int frame_count_;
};
