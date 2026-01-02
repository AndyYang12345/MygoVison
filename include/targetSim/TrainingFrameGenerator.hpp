#ifndef TRAINING_FRAME_GENERATOR_HPP
#define TRAINING_FRAME_GENERATOR_HPP

#include "targetSim.hpp"
#include <functional>
#include <vector>
#include <memory>

/**
 * @class TrainingFrameGenerator
 * @brief Training frame generator - designed for PID tuning and algorithm training
 */
class TrainingFrameGenerator {
public:
    // Training modes
    enum TrainingMode {
        MODE_PENTAGON_ROTATION,    // Pentagon rotation mode
        MODE_LINEAR_MOVEMENT,      // Linear movement mode
        MODE_RANDOM_APPEARANCE,    // Random appearance mode
        MODE_PARAMETRIC_MOTION     // Parametric motion mode
    };
    
    // 定义参数方程类型
    using ParametricFunction = std::function<float(float t)>;
    // 角速度函数类型
    using AngularVelocityFunction = std::function<float(float t)>;
    
    /**
     * @brief Constructor
     * @param width Image width
     * @param height Image height
     * @param fps Frame rate
     */
    TrainingFrameGenerator(int width = 800, int height = 600, float fps = 30.0f);
    
    /**
     * @brief 设置训练模式
     * @param mode 训练模式 
     * @param param1 模式参数 1
     * @param param2 模式参数 2
     * @param angular_velocity_func 角速度函数（仅对旋转模式有效）
     */
    void set_training_mode(TrainingMode mode, float param1 = 0.0f, float param2 = 0.0f, AngularVelocityFunction angular_velocity_func = nullptr);
    
    /**
     * @brief 设置参数方程运动模式
     * @param x_func x坐标关于时间的函数: x = f(t)
     * @param y_func y坐标关于时间的函数: y = f(t)
     * @param duration 运动周期时长（秒）
     * @param loop 是否循环运动
     */
    void set_parametric_motion_mode(ParametricFunction x_func, 
                                   ParametricFunction y_func,
                                   float duration = 10.0f,
                                   bool loop = true);
    
    /**
     * @brief 预定义参数方程：圆周运动
     * @param center 圆心
     * @param radius 半径
     * @param angular_speed 角速度（弧度/秒）
     * @param loop 是否循环运动
     */
    void set_circular_motion(const cv::Point2f& center, float radius, 
                            float angular_speed, bool loop = true);
    
    /**
     * @brief 预定义参数方程：正弦运动
     * @param start_point 起始点
     * @param amplitude 振幅
     * @param frequency 频率（Hz）
     * @param direction 运动方向（角度，0度表示水平向右）
     */
    void set_sine_motion(const cv::Point2f& start_point, float amplitude, 
                        float frequency, float direction = 0.0f, float speed = 50.0f, bool loop = true, float duration = 10.0f);
    
    /**
     * @brief 预定义参数方程：Lissajous曲线
     * @param center 中心点
     * @param a x方向振幅
     * @param b y方向振幅
     * @param wx x方向频率（弧度/秒）
     * @param wy y方向频率（弧度/秒）
     * @param phase 相位差
     */
    void set_lissajous_motion(const cv::Point2f& center, float a, float b,
                             float wx, float wy, float phase = 0.0f, bool loop = true, float duration = 10.0f);
    
    /**
     * @brief Get next training frame
     * @param timestamp Timestamp (seconds), if -1 use internal time
     * @return Training frame and ground truth target position
     */
    struct TrainingFrame {
        cv::Mat frame;
        cv::Point2f target_position;   // Ground truth target position (pixels)
        float timestamp;
        cv::Point2f velocity;          // 速度向量
    };
    
    /**
     * @brief Get next training frame with optional position control
     * @param timestamp Timestamp (seconds), if -1 use internal time
     * @param pentagon_center 五角星中心位置，如果为(-1,-1)则使用当前中心
     * @return Training frame and ground truth target position
     */
    TrainingFrame get_next_frame(float timestamp = -1.0f, 
                                const cv::Point2f& pentagon_center = cv::Point2f(-1, -1));
    
    /**
     * @brief 强制重新生成五角星（新位置和颜色）
     * @param center 新的中心位置，如果为(-1,-1)则随机位置
     */
    void regenerate_pentagon(const cv::Point2f& center = cv::Point2f(-1, -1));

    /**
     * @brief Reset generator
     */
    void reset();
    
    /**
     * @brief Get current time
     */
    float get_current_time() const { return _current_time; }
    
    /**
     * @brief Get current mode
     */
    TrainingMode get_current_mode() const { return _current_mode; }
    
    /**
     * @brief Pause time progression
     */
    void pause() { _paused = true; }
    
    /**
     * @brief Resume time progression
     */
    void resume() { _paused = false; }
    
    /**
     * @brief Check if generator is paused
     */
    bool is_paused() const { return _paused; }
    
    /**
     * @brief Manually set current time
     */
    void set_current_time(float time) { _current_time = time; }
    
    /**
     * @brief 获取靶子仿真器的中心位置
     */
    cv::Point2f get_target_sim_center() const { return _target_sim.get_center(); }
    
    /**
     * @brief 设置是否循环运动（仅对参数方程模式有效）
     */
    void set_loop_motion(bool loop) { _loop_motion = loop; }
    
    /**
     * @brief 获取当前运动是否循环
     */
    bool is_loop_motion() const { return _loop_motion; }

    /**
     * @brief 获取当前五角星中心位置
     */
    cv::Point2f get_current_pentagon_center() const { return _current_pentagon_center; }

private:
    TargetSim _target_sim;
    float _fps;
    float _current_time;
    TrainingMode _current_mode;
    bool _paused;
    
    // Mode parameters
    float _param1, _param2;
    
    // 角速度函数成员
    AngularVelocityFunction _angular_velocity_func;
    
    // 参数方程相关成员
    ParametricFunction _x_function;
    ParametricFunction _y_function;
    float _parametric_duration;
    bool _loop_motion;
    
    // Internal state
    cv::Point2f _current_position;
    float _last_appear_time;
    cv::Point2f _last_random_position;
    private:

    // 五角星旋转模式状态
    cv::Point2f _current_pentagon_center;
    bool _need_regenerate_pentagon;
    float _current_pentagon_angle;
    
    // Mode processing functions
    TrainingFrame _generate_pentagon_rotation(float timestamp);
    TrainingFrame _generate_linear_movement(float timestamp);
    TrainingFrame _generate_random_appearance(float timestamp);
    TrainingFrame _generate_parametric_motion(float timestamp);
    
    // 辅助函数
    cv::Point2f _calculate_parametric_position(float t);
    cv::Point2f _calculate_parametric_velocity(float t, float dt = 0.01f);
};

#endif // TRAINING_FRAME_GENERATOR_HPP