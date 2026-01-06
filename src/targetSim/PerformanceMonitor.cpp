#include "targetSim/PerformanceMonitor.hpp"
#include <iostream>

PerformanceMonitor::PerformanceMonitor() {
    reset();
}

// 开始帧计时
void PerformanceMonitor::begin_frame() {
    _frame_start = std::chrono::high_resolution_clock::now();
}

// 标记生成完成
void PerformanceMonitor::mark_generation_done() {
    _generation_done = std::chrono::high_resolution_clock::now();
}

// 标记渲染完成
void PerformanceMonitor::mark_rendering_done() {
    _rendering_done = std::chrono::high_resolution_clock::now();
}

// 结束帧并获取指标
PerformanceMonitor::FrameMetrics PerformanceMonitor::end_frame() {
    auto frame_end = std::chrono::high_resolution_clock::now();
    
    FrameMetrics metrics;
    metrics.generation_time_ms = std::chrono::duration<float, std::milli>(
        _generation_done - _frame_start).count();
    metrics.rendering_time_ms = std::chrono::duration<float, std::milli>(
        _rendering_done - _generation_done).count();
    metrics.wait_time_ms = std::chrono::duration<float, std::milli>(
        frame_end - _rendering_done).count();
    metrics.frame_time_ms = metrics.generation_time_ms + 
                           metrics.rendering_time_ms + 
                           metrics.wait_time_ms;
    
    // 计算FPS
    _frame_count++;
    float elapsed = std::chrono::duration<float>(
        frame_end - _last_fps_time).count();
    
    if (elapsed >= 1.0f) {
        metrics.fps = _frame_count / elapsed;
        _frame_count = 0;
        _last_fps_time = frame_end;
    } else {
        metrics.fps = _current_fps;
    }
    _current_fps = metrics.fps;
    
    return metrics;
}

void PerformanceMonitor::reset() {
    _frame_count = 0;
    _current_fps = 0;
    _last_fps_time = std::chrono::high_resolution_clock::now();
}