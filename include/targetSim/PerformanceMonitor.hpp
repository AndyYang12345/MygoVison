#ifndef PERFORMANCE_MONITOR_HPP
#define PERFORMANCE_MONITOR_HPP

#include <chrono>

/**
 * @class PerformanceMonitor
 * @brief 性能监视器 - 用于监视帧生成和渲染的性能指标
 */
class PerformanceMonitor {
public:
    struct FrameMetrics {
        float frame_time_ms;      // 总帧时间
        float generation_time_ms; // 帧生成时间
        float rendering_time_ms;  // 渲染时间
        float wait_time_ms;       // 等待时间
        float fps;               // 当前FPS
    };
    
    PerformanceMonitor();
    
    // 开始帧计时
    void begin_frame();
    
    // 标记生成完成
    void mark_generation_done();
    
    // 标记渲染完成
    void mark_rendering_done();
    
    // 结束帧并获取指标
    FrameMetrics end_frame();
    
    void reset();
    
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> _frame_start;
    std::chrono::time_point<std::chrono::high_resolution_clock> _generation_done;
    std::chrono::time_point<std::chrono::high_resolution_clock> _rendering_done;
    std::chrono::time_point<std::chrono::high_resolution_clock> _last_fps_time;
    int _frame_count;
    float _current_fps;
};

#endif // PERFORMANCE_MONITOR_HPP