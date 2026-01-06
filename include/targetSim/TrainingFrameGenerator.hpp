#ifndef TRAINING_FRAME_GENERATOR_HPP
#define TRAINING_FRAME_GENERATOR_HPP

#include "targetSim.hpp"
#include <functional>
#include <vector>
#include <memory>

/**
 * @class TrainingFrameGenerator 用于生成训练帧
 * @brief Training frame generator - 用于生成包含靶子图像的训练帧，可以根据传入函数生成不同运动轨迹
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
    
    // 性能数据结构（需要在类声明中定义）
    struct FrameMetrics {
        float frame_time_ms;
        float generation_time_ms;
        float rendering_time_ms;
        float wait_time_ms;
        float fps;
    };
    
    /**
     * @brief 训练帧结构体，包含图像和Ground Truth信息
     */
    struct TrainingFrame {
        cv::Mat frame;              // 生成的图像帧
        cv::Point2f target_position;   // Ground Truth目标位置
        float timestamp;               // 帧时间戳（秒）
        cv::Point2f velocity;          // 速度向量
        
        // 新增性能指标字段
        float actual_fps;              // 实际帧率
        float frame_time_ms;           // 帧处理总时间（毫秒）
        float generation_time_ms;      // 帧生成时间（毫秒）
        float rendering_time_ms;       // 帧渲染时间（毫秒）
    };
    
    /**
     * @brief 构造函数
     * @param width 生成图像的宽度（像素）
     * @param height 生成图像的高度（像素）
     * @param fps 帧率（帧/秒）
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
     * @brief 获取下一个训练帧，带可选位置控制
     * @param timestamp 时间戳（秒），如果为-1则使用内部时间
     * @param pentagon_center 五角星中心位置，如果为(-1,-1)则使用当前中心
     * @return 训练帧和Ground Truth目标位置
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
     * @brief 获取靶子仿真器的中心位置，也就是整个图像的中心位置
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
    
    // 五角星旋转模式状态
    cv::Point2f _current_pentagon_center;
    bool _need_regenerate_pentagon;
    float _current_pentagon_angle;
    
    // 性能监控数据
    struct PerformanceData {
        std::chrono::time_point<std::chrono::high_resolution_clock> frame_start;
        std::chrono::time_point<std::chrono::high_resolution_clock> generation_done;
        std::chrono::time_point<std::chrono::high_resolution_clock> rendering_done;
        std::chrono::time_point<std::chrono::high_resolution_clock> last_fps_time;
        int frame_count;
        float current_fps;
        float avg_fps;
        float min_fps;
        float max_fps;
        int samples;
    } _perf_data;
    bool _performance_monitoring;
    
    // Mode processing functions
    TrainingFrame _generate_pentagon_rotation(float timestamp);
    TrainingFrame _generate_linear_movement(float timestamp);
    TrainingFrame _generate_random_appearance(float timestamp);
    TrainingFrame _generate_parametric_motion(float timestamp);
    
    // 辅助函数
    cv::Point2f _calculate_parametric_position(float t);
    cv::Point2f _calculate_parametric_velocity(float t, float dt = 0.01f);
    
    // 性能监控辅助函数
    void _start_frame_timing();
    void _mark_generation_done();
    void _mark_rendering_done();
    FrameMetrics _end_frame_timing();  // 移除了PerformanceData::前缀
};

#endif // TRAINING_FRAME_GENERATOR_HPP