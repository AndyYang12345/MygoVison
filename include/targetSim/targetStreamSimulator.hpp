// targetStreamSimulator.hpp - 修正版本
#ifndef TARGET_STREAM_SIMULATOR_HPP
#define TARGET_STREAM_SIMULATOR_HPP

#include "targetSim.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <functional>

class TargetStreamSimulator {
public:
    using PositionFunction = std::function<cv::Point2f(float)>;
    using RotationFunction = std::function<float(float)>;
    using ColorFunction = std::function<std::vector<cv::Scalar>(float)>;

    TargetStreamSimulator(int width = 800, int height = 600, 
                         float fps = 30.0f,
                         cv::Scalar background = cv::Scalar(220, 220, 220));
    
    void configure_basic(const cv::Scalar& center_color = cv::Scalar(0, 0, 255),
                        const std::string& pattern = "pentagon",
                        int target_idx = 0);
    
    void configure_fixed_color_motion(
        const std::vector<cv::Scalar>& surround_colors,
        RotationFunction rotation_func = nullptr,
        PositionFunction position_func = nullptr);
    
    void configure_random_color_motion(
        RotationFunction rotation_func = nullptr,
        PositionFunction position_func = nullptr,
        float color_change_rate = 0.0f);
    
    void configure_custom_color_motion(
        ColorFunction color_func,
        RotationFunction rotation_func = nullptr,
        PositionFunction position_func = nullptr);
    
    void configure_single_blob_motion(
        const cv::Scalar& color,
        PositionFunction position_func,
        int size = 30);
    
    // 预定义运动函数
    static PositionFunction linear_motion(cv::Point2f start, cv::Point2f velocity);
    static PositionFunction circular_motion(cv::Point2f center, float radius, float angular_speed);
    static PositionFunction sinusoidal_motion(cv::Point2f start, float amplitude, float frequency);
    static RotationFunction constant_rotation(float speed);
    static RotationFunction oscillating_rotation(float amplitude, float frequency);
    
    cv::Mat get_frame_at_time(float timestamp);
    cv::Mat get_next_frame();
    void reset();
    
    // Getter函数 - 实现这些！
    float get_current_time() const { return _current_time; }
    cv::Point2f get_current_center() const { return _current_center; }
    float get_current_rotation() const { return _current_rotation; }
    
    // 新增：获取当前颜色
    std::vector<cv::Scalar> get_current_colors() const { 
        if (_color_func) {
            return _color_func(_current_time);
        } else if (_use_random_colors) {
            return _current_random_colors;
        }
        // 返回默认颜色
        return {
            cv::Scalar(0, 255, 0),
            cv::Scalar(255, 0, 0),
            cv::Scalar(255, 255, 0),
            cv::Scalar(255, 0, 255),
            cv::Scalar(0, 255, 255)
        };
    }
    
    // 获取画布尺寸
    int get_width() const { return _canvas_width; }
    int get_height() const { return _canvas_height; }
    
    TargetSim& get_target_sim() { return _target_sim; }

private:
    TargetSim _target_sim;
    float _fps;
    float _current_time;
    
    // 画布参数
    int _canvas_width;
    int _canvas_height;
    cv::Scalar _canvas_background;
    
    // 配置参数
    std::string _pattern;
    cv::Scalar _center_color;
    int _target_idx;
    
    // 运动函数
    PositionFunction _position_func;
    RotationFunction _rotation_func;
    ColorFunction _color_func;
    
    // 当前状态（需要更新！）
    cv::Point2f _current_center;    // 当前中心位置
    float _current_rotation;        // 当前旋转角度
    
    // 单色块参数
    bool _is_single_blob;
    int _blob_size;
    
    // 随机颜色参数
    bool _use_random_colors;
    float _color_change_rate;
    float _last_color_change_time;
    std::vector<cv::Scalar> _current_random_colors;
    
    cv::Mat _generate_frame_for_pattern(float timestamp);
    cv::Mat _generate_single_blob(float timestamp);
    cv::Mat _generate_multi_blob(float timestamp);
    void _update_random_colors(float timestamp);
    
    // 新增：更新当前状态的函数
    void _update_current_state(float timestamp);
};

#endif // TARGET_STREAM_SIMULATOR_HPP