#ifndef PERFORMANCE_MONITOR_HPP
#define PERFORMANCE_MONITOR_HPP

#include <chrono>

/**
 * @class PerformanceMonitor
 * @brief 简化版性能监视器 - 只提供FPS计算功能
 */
class PerformanceMonitor {
public:
    /**
     * @brief 构造函数
     */
    PerformanceMonitor();
    
    /**
     * @brief 每帧调用，更新计时
     * @return 当前的FPS值（如果满1秒则更新，否则返回上次的FPS）
     */
    float tick();
    
    /**
     * @brief 获取当前FPS
     */
    float get_fps() const { return _current_fps; }
    
    /**
     * @brief 重置计数器
     */
    void reset();
    
    /**
     * @brief 获取平均FPS
     */
    float get_average_fps() const { return _avg_fps; }
    
    /**
     * @brief 获取最小FPS
     */
    float get_min_fps() const { return _min_fps; }
    
    /**
     * @brief 获取最大FPS
     */
    float get_max_fps() const { return _max_fps; }
    
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> _last_time;
    int _frame_count;
    float _current_fps;
    float _avg_fps;
    float _min_fps;
    float _max_fps;
    int _samples;
};

#endif // PERFORMANCE_MONITOR_HPP