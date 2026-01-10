#include "PerformanceMonitor.hpp"
#include <iostream>

PerformanceMonitor::PerformanceMonitor() {
    reset();
}

float PerformanceMonitor::tick() {
    auto now = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float>(now - _last_time).count();
    _frame_count++;
    
    if (elapsed >= 1.0f) {
        _current_fps = _frame_count / elapsed;
        _frame_count = 0;
        _last_time = now;
        
        // 更新统计
        _samples++;
        _avg_fps = (_avg_fps * (_samples - 1) + _current_fps) / _samples;
        _min_fps = std::min(_min_fps, _current_fps);
        _max_fps = std::max(_max_fps, _current_fps);
    }
    
    return _current_fps;
}

void PerformanceMonitor::reset() {
    _frame_count = 0;
    _current_fps = 0.0f;
    _avg_fps = 0.0f;
    _min_fps = 9999.0f;
    _max_fps = 0.0f;
    _samples = 0;
    _last_time = std::chrono::high_resolution_clock::now();
}