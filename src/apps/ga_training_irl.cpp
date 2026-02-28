#include "TargetTracking/GeneticAlgorithm.hpp"
#include "TargetTracking/GimbalControl.hpp"
#include "TargetTracking/TargetTracker.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

struct Sample {
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
    float dt = 0.033f;
};

struct GimbalFeedback {
    bool valid = false;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
};

struct PidState {
    float integral = 0.0f;
    float prev_error = 0.0f;
    bool has_prev = false;
};

float clamp_value(float v, float lo, float hi) {
    return std::max(lo, std::min(v, hi));
}

GimbalFeedback read_gimbal_feedback_placeholder() {
    return {};
}

float step_pid(float target, float current, float dt, const Genome& g, PidState& state,
               float integral_limit, float max_speed_deg) {
    float error = target - current;
    state.integral += error * dt;
    state.integral = clamp_value(state.integral, -integral_limit, integral_limit);

    float derivative = 0.0f;
    if (state.has_prev && dt > 1e-6f) {
        derivative = (error - state.prev_error) / dt;
    }
    state.prev_error = error;
    state.has_prev = true;

    float speed = g.p * error + g.i * state.integral + g.d * derivative;
    speed = clamp_value(speed, -max_speed_deg, max_speed_deg);
    return current + speed * dt;
}

float evaluate_genome_on_samples(const Genome& g,
                                 const std::vector<Sample>& samples,
                                 float integral_limit,
                                 float max_speed_deg,
                                 float w_error,
                                 float w_smooth,
                                 float w_energy) {
    if (samples.empty()) {
        return -1e9f;
    }

    float current_pitch = 0.0f;
    float current_yaw = 0.0f;
    float total_error = 0.0f;
    float total_smooth = 0.0f;
    float total_energy = 0.0f;

    float prev_speed_pitch = 0.0f;
    float prev_speed_yaw = 0.0f;

    PidState pitch_state;
    PidState yaw_state;

    for (const auto& s : samples) {
        float dt = clamp_value(s.dt, 0.001f, 0.05f);

        float prev_pitch = current_pitch;
        float prev_yaw = current_yaw;

        current_pitch = step_pid(s.pitch_deg, current_pitch, dt, g, pitch_state, integral_limit, max_speed_deg);
        current_yaw = step_pid(s.yaw_deg, current_yaw, dt, g, yaw_state, integral_limit, max_speed_deg);

        float speed_pitch = (current_pitch - prev_pitch) / dt;
        float speed_yaw = (current_yaw - prev_yaw) / dt;

        float err_pitch = std::abs(s.pitch_deg - current_pitch);
        float err_yaw = std::abs(s.yaw_deg - current_yaw);
        total_error += (err_pitch + err_yaw) * dt;

        total_smooth += (std::abs(speed_pitch - prev_speed_pitch) +
                         std::abs(speed_yaw - prev_speed_yaw)) * dt;

        total_energy += (std::abs(speed_pitch) + std::abs(speed_yaw)) * dt;

        prev_speed_pitch = speed_pitch;
        prev_speed_yaw = speed_yaw;
    }

    float cost = w_error * total_error +
                 w_smooth * total_smooth +
                 w_energy * total_energy;

    return -cost;
}

std::vector<Sample> collect_samples_from_camera(int camera_id,
                                                int max_frames,
                                                bool show_preview,
                                                float fx_hint,
                                                float fy_hint,
                                                GimbalControl* gimbal,
                                                float pitch_home_deg,
                                                float yaw_home_deg,
                                                float command_speed_deg) {
    std::vector<Sample> samples;

    cv::VideoCapture cap(camera_id);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera id " << camera_id << std::endl;
        return samples;
    }

    TargetTracker tracker;
    TrackerConfig tracker_cfg = tracker.get_config();
    tracker_cfg.show_debug_windows = false;
    tracker_cfg.print_debug_info = false;
    tracker.set_config(tracker_cfg);

    const std::string win_name = "GA IRL Capture";
    if (show_preview) {
        cv::namedWindow(win_name, cv::WINDOW_AUTOSIZE);
    }

    auto last_tick = std::chrono::steady_clock::now();
    int collected = 0;
    int found_count = 0;

    while (collected < max_frames) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            continue;
        }

        auto now_tick = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now_tick - last_tick).count();
        last_tick = now_tick;
        dt = clamp_value(dt, 0.001f, 0.05f);

        float fx = fx_hint > 0.0f ? fx_hint : frame.cols * 0.6f;
        float fy = fy_hint > 0.0f ? fy_hint : frame.rows * 0.6f;
        float cx = frame.cols * 0.5f;
        float cy = frame.rows * 0.5f;

        TargetInfo info = tracker.process_frame(frame);
        if (info.found) {
            float dx = info.target_center.x - cx;
            float dy = info.target_center.y - cy;
            float target_pitch_deg = std::atan2(dy, fy) * 180.0f / static_cast<float>(CV_PI);
            float target_yaw_deg = -std::atan2(dx, fx) * 180.0f / static_cast<float>(CV_PI);

            if (gimbal != nullptr && gimbal->is_serial_open()) {
                float command_pitch = clamp_value(pitch_home_deg + target_pitch_deg, 0.0f, 180.0f);
                float command_yaw = clamp_value(yaw_home_deg + target_yaw_deg, 0.0f, 270.0f);
                gimbal->set_pitch_angle(command_pitch);
                gimbal->set_yaw_angle(command_yaw);
                gimbal->set_pitch_speed(command_speed_deg);
                gimbal->set_yaw_speed(command_speed_deg);
                gimbal->get_command();
                gimbal->send_command();
            }

            GimbalFeedback feedback = read_gimbal_feedback_placeholder();
            float measured_pitch_deg = 0.0f;
            float measured_yaw_deg = 0.0f;
            if (feedback.valid) {
                measured_pitch_deg = feedback.pitch_deg - pitch_home_deg;
                measured_yaw_deg = feedback.yaw_deg - yaw_home_deg;
            }

            Sample s;
            s.pitch_deg = target_pitch_deg - measured_pitch_deg;
            s.yaw_deg = target_yaw_deg - measured_yaw_deg;
            s.dt = dt;
            samples.push_back(s);
            found_count++;

            if (show_preview) {
                cv::circle(frame, info.target_center, 6, cv::Scalar(0, 0, 255), -1);
                cv::drawMarker(frame, cv::Point(static_cast<int>(cx), static_cast<int>(cy)),
                               cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 12, 2);
                std::string msg = "pitch=" + std::to_string(s.pitch_deg).substr(0, 6) +
                                  " yaw=" + std::to_string(s.yaw_deg).substr(0, 6);
                cv::putText(frame, msg, cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX,
                            0.6, cv::Scalar(255, 255, 255), 2);
            }
        }

        if (show_preview) {
            std::string line = "Frame " + std::to_string(collected + 1) + "/" + std::to_string(max_frames) +
                               " | Found " + std::to_string(found_count);
            cv::putText(frame, line, cv::Point(12, frame.rows - 14), cv::FONT_HERSHEY_SIMPLEX,
                        0.55, cv::Scalar(200, 200, 200), 1);
            cv::imshow(win_name, frame);
            int key = cv::waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q') {
                break;
            }
        }

        collected++;
    }

    if (show_preview) {
        cv::destroyWindow(win_name);
    }

    std::cout << "Collected samples: " << samples.size() << " / " << max_frames << std::endl;
    return samples;
}

} // namespace

int main(int argc, char** argv) {
    int camera_id = 0;
    int collect_frames = 600;
    int population_size = 50;
    int generations = 200;
    bool show_preview = true;
    bool enable_gimbal = true;
    std::string serial_device = "/dev/ttyUSB0";
    int serial_baud = 115200;
    float pitch_home_deg = 60.0f;
    float yaw_home_deg = 105.0f;
    float command_speed_deg = 180.0f;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--cam" && i + 1 < argc) {
            camera_id = std::stoi(argv[++i]);
        } else if (arg == "--frames" && i + 1 < argc) {
            collect_frames = std::stoi(argv[++i]);
        } else if (arg == "--pop" && i + 1 < argc) {
            population_size = std::stoi(argv[++i]);
        } else if (arg == "--gen" && i + 1 < argc) {
            generations = std::stoi(argv[++i]);
        } else if (arg == "--no-preview") {
            show_preview = false;
        } else if (arg == "--serial" && i + 1 < argc) {
            serial_device = argv[++i];
        } else if (arg == "--baud" && i + 1 < argc) {
            serial_baud = std::stoi(argv[++i]);
        } else if (arg == "--no-gimbal") {
            enable_gimbal = false;
        }
    }

    std::cout << "[GA-IRL] camera=" << camera_id
              << " frames=" << collect_frames
              << " pop=" << population_size
              << " gen=" << generations
              << " gimbal=" << (enable_gimbal ? "on" : "off")
              << " serial=" << serial_device
              << " baud=" << serial_baud << std::endl;

    GimbalControl gimbal;
    GimbalControl* gimbal_ptr = nullptr;
    if (enable_gimbal) {
        if (gimbal.open_serial(serial_device, serial_baud)) {
            gimbal_ptr = &gimbal;
            std::cout << "Gimbal serial opened." << std::endl;
        } else {
            std::cerr << "Failed to open gimbal serial, continue without gimbal output." << std::endl;
        }
    }

    const auto samples = collect_samples_from_camera(camera_id, collect_frames, show_preview,
                                                     -1.0f, -1.0f,
                                                     gimbal_ptr,
                                                     pitch_home_deg, yaw_home_deg,
                                                     command_speed_deg);
    if (samples.size() < 30) {
        std::cerr << "Not enough valid samples for GA training." << std::endl;
        if (gimbal_ptr != nullptr) {
            gimbal.close_serial();
        }
        return 1;
    }

    Population population(population_size, 20260228);
    population.set_mutation(0.35f, 0.2f);
    population.set_elitism(2);

    const float integral_limit = 30.0f;
    const float max_speed_deg = 180.0f;
    const float w_error = 1.0f;
    const float w_smooth = 0.05f;
    const float w_energy = 0.01f;

    population.set_fitness_function([&](const Genome& g) {
        return evaluate_genome_on_samples(g, samples, integral_limit, max_speed_deg,
                                          w_error, w_smooth, w_energy);
    });

    Genome base;
    population.initialize_random(base);

    for (int gen = 0; gen < generations; ++gen) {
        population.evaluate_all();
        const Individual& best = population.best();
        std::cout << "Generation " << population.generation()
                  << " | Best Fitness: " << best.fitness
                  << " | P: " << best.genome.p
                  << ", I: " << best.genome.i
                  << ", D: " << best.genome.d << std::endl;

        if (gen + 1 < generations) {
            population.evolve_next();
        }
    }

    population.evaluate_all();
    const Individual& best = population.best();
    std::cout << "Best PID (IRL samples) => P: " << best.genome.p
              << ", I: " << best.genome.i
              << ", D: " << best.genome.d
              << ", Fitness: " << best.fitness << std::endl;

    if (gimbal_ptr != nullptr) {
        gimbal.close_serial();
    }

    return 0;
}
