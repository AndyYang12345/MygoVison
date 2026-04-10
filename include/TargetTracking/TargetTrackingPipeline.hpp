#ifndef TARGET_TRACKING_PIPELINE_HPP
#define TARGET_TRACKING_PIPELINE_HPP

#include <opencv2/opencv.hpp>
#include <string>

#include "TargetTracking/GimbalControl.hpp"
#include "TargetTracking/TargetTracker.hpp"

enum class TrackState { Waiting, Searching, Locked, Tracking };

struct PipelineConfig {
    float fx = -1.0f;
    float fy = -1.0f;
    float cx = -1.0f;
    float cy = -1.0f;

    float pitch_home = 135.0f;
    float yaw_home = 135.0f;

    float pitch_pwm_zero_angle = 135.0f;
    float yaw_pwm_zero_angle = 135.0f;

    float max_speed = 180.0f;
    float integral_limit = 30.0f;
    float pid_kp = 11.8354f;
    float pid_ki = 0.315478f;
    float pid_kd = 0.0215511f;

    float pitch_error_sign = -1.0f;
    float yaw_error_sign = 1.0f;

    int lock_required = 30;
    int lost_required = 10;

    float scan_yaw_amp = 30.0f;
    float scan_pitch_amp = 15.0f;
    float scan_yaw_freq = 0.15f;
    float scan_pitch_freq = 0.10f;
    float scan_yaw_phase = 0.0f;
    float scan_pitch_phase = 0.0f;

    bool draw_overlay = true;
    bool print_debug = false;
    bool control_enabled = true;

    bool enable_serial = false;
    std::string serial_device = "/dev/ttyUSB0";
    int serial_baud = 115200;
};

struct PipelineOutput {
    cv::Mat canvas;
    std::string command;
    TrackState state = TrackState::Waiting;
    int lock_count = 0;
    int lost_count = 0;
    bool target_found = false;
    cv::Point2f target_pos{-1.0f, -1.0f};
    bool roi_active = false;
    cv::Rect roi_rect{-1, -1, 0, 0};
    bool laser_found = false;
    cv::Point2f laser_pos{-1.0f, -1.0f};
    cv::Point2f aim_pos{-1.0f, -1.0f};
    bool aim_from_laser = false;
    float laser_target_error_px = 0.0f;
    float pitch_angle = 0.0f;
    float yaw_angle = 0.0f;
    float pitch_speed = 0.0f;
    float yaw_speed = 0.0f;
};

class TargetTrackingPipeline {
public:
    TargetTrackingPipeline();

    void set_config(const PipelineConfig& config);
    PipelineConfig get_config() const;

    void set_tracker_config(const TrackerConfig& config);
    TrackerConfig get_tracker_config() const;

    void set_control_enabled(bool enabled);
    void start_tracking();

    void reset();
    void handle_key(int key);

    PipelineOutput process_frame(const cv::Mat& frame, float dt);

    float get_pitch_angle() const;
    float get_yaw_angle() const;

    bool open_serial();
    void close_serial();
    bool is_serial_open() const;
    bool send_raw_serial_command(const std::string& command);

private:
    struct PID {
        float kp{0.0f};
        float ki{0.0f};
        float kd{0.0f};
        float integral{0.0f};
        float prev_error{0.0f};
        bool has_prev{false};
    };

    float clamp_value(float v, float lo, float hi) const;
    float pid_step(float error, float dt, PID& pid, float integral_limit);
    void reset_pid(PID& pid);
    void apply_pid_gains_from_config();

    PipelineConfig config_;
    bool control_enabled_ = true;
    TrackState state_ = TrackState::Waiting;
    int lock_count_ = 0;
    int lost_count_ = 0;
    float scan_time_ = 0.0f;

    float pitch_angle_ = 0.0f;
    float yaw_angle_ = 0.0f;
    float pitch_speed_ = 0.0f;
    float yaw_speed_ = 0.0f;

    PID pid_pitch_;
    PID pid_yaw_;

    GimbalControl gimbal_;
    TargetTracker tracker_;
};

#endif // TARGET_TRACKING_PIPELINE_HPP
