#include "TargetTracking/GeneticAlgorithm.hpp"
#include "TargetSim/Target3DGenerator.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#define POPULATION_SIZE 50
#define MUTATION_RATE 0.5f
#define MUTATION_SIGMA 0.3f
#define MAX_GENERATIONS 300
#define ELITE_COUNT 2

// 修改这里来指定需要展示的代数
static const std::vector<int> kDisplayGenerations = {299};

struct DisplayMethods {
    int random = 2;
    int sine = 2;
    int circular = 2;
    int lissajous = 2;
    float duration_sec = 4.0f;
};

// 修改这里来指定展示的测试次数
static const DisplayMethods kDisplayMethods{};

namespace {
struct PidState {
    float integral = 0.0f;
    float prev_error = 0.0f;
    bool has_prev = false;
};

float clamp_value(float v, float lo, float hi) {
    return std::max(lo, std::min(v, hi));
}

float step_pid(float target, float current, float dt, const Genome& g, PidState& state,
               float max_rate) {
    float error = target - current;
    state.integral += error * dt;
    state.integral = clamp_value(state.integral, -1.0f, 1.0f);

    float derivative = 0.0f;
    if (state.has_prev && dt > 1e-6f) {
        derivative = (error - state.prev_error) / dt;
    }
    state.prev_error = error;
    state.has_prev = true;

    float rate = g.p * error + g.i * state.integral + g.d * derivative;
    rate = clamp_value(rate, -max_rate, max_rate);
    return current + rate * dt;
}

bool should_display_generation(int gen) {
    return std::find(kDisplayGenerations.begin(), kDisplayGenerations.end(), gen)
           != kDisplayGenerations.end();
}

void run_visualization(const Genome& g, int generation_index) {
    const int width = 640;
    const int height = 480;
    Target3DGenerator generator(width, height, 30.0f);

    SimulationCamera::CameraIntrinsics intr;
    intr.fx = width * 0.9f;
    intr.fy = width * 0.9f;
    intr.cx = width * 0.5f;
    intr.cy = height * 0.5f;
    generator.set_camera_intrinsics(intr);

    SimulationCamera::CameraPose pose;
    pose.position = cv::Point3f(0.0f, 0.0f, -600.0f);
    pose.rotation = cv::Point3f(0.0f, 0.0f, 0.0f);
    generator.set_camera_pose(pose);
    generator.set_target_plane(cv::Point3f(0.0f, 0.0f, 1.0f), 600.0f);

    cv::namedWindow("GA Best Tracking", cv::WINDOW_AUTOSIZE);

    const float max_rate = 600.0f; // pixels/s
    const float settle_threshold = 5.0f;
    const float settle_hold = 0.2f;
    const float target_interval = 0.6f;
    generator.set_random_interval(target_interval);
    generator.generator().pause();

    const cv::Point2f center(static_cast<float>(width) * 0.5f, static_cast<float>(height) * 0.5f);
    const float amp_x = width * 0.25f;
    const float amp_y = height * 0.2f;
    const float omega_sine = 2.0f * static_cast<float>(M_PI) * 0.4f;
    const float omega_circle = 2.0f * static_cast<float>(M_PI) * 0.35f;
    const float w_liss_x = 1.1f;
    const float w_liss_y = 1.7f;

    auto run_random_trial = [&](int trial_idx, int total_trials) -> bool {
        cv::Point2f aim = center;
        PidState pid_x;
        PidState pid_y;
        bool tracking_active = false;
        bool request_new_target = true;
        cv::Point2f target_pos(-1.0f, -1.0f);
        float settle_timer = 0.0f;

        auto last_tick = std::chrono::steady_clock::now();
        while (true) {
            auto now_tick = std::chrono::steady_clock::now();
            float dt = std::chrono::duration<float>(now_tick - last_tick).count();
            last_tick = now_tick;
            dt = clamp_value(dt, 0.001f, 0.05f);

            if (request_new_target) {
                float current_time = generator.generator().get_current_time();
                generator.generator().set_current_time(current_time + target_interval + 0.01f);
                request_new_target = false;
            }

            auto result = generator.generate_projected_frame();
            cv::Mat frame = result.projected_frame.clone();

            if (!result.projected_positions.empty()) {
                target_pos = result.projected_positions[0];
                tracking_active = true;
            }

            if (tracking_active && target_pos.x >= 0.0f) {
                aim.x = step_pid(target_pos.x, aim.x, dt, g, pid_x, max_rate);
                aim.y = step_pid(target_pos.y, aim.y, dt, g, pid_y, max_rate);

                float dx = std::abs(target_pos.x - aim.x);
                float dy = std::abs(target_pos.y - aim.y);
                if (dx < settle_threshold && dy < settle_threshold) {
                    settle_timer += dt;
                    if (settle_timer >= settle_hold) {
                        break;
                    }
                } else {
                    settle_timer = 0.0f;
                }
            }

            if (!result.projected_positions.empty()) {
                cv::circle(frame, result.projected_positions[0], 6, cv::Scalar(0, 0, 255), -1);
            }
            cv::drawMarker(frame, aim, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 16, 2);

            std::string title = "Gen " + std::to_string(generation_index) +
                                " | Random " + std::to_string(trial_idx) + "/" +
                                std::to_string(total_trials);
            cv::putText(frame, title, cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX,
                        0.7, cv::Scalar(255, 255, 255), 2);
            cv::imshow("GA Best Tracking", frame);

            int key = cv::waitKey(16);
            if (key == 27 || key == 'q' || key == 'Q') {
                return false;
            }
        }
        return true;
    };

    auto run_motion_trial = [&](const std::string& name,
                                int trial_idx,
                                int total_trials,
                                const std::function<cv::Point2f(float)>& target_func) -> bool {
        cv::Point2f aim = center;
        PidState pid_x;
        PidState pid_y;
        float elapsed = 0.0f;

        auto last_tick = std::chrono::steady_clock::now();
        while (elapsed < kDisplayMethods.duration_sec) {
            auto now_tick = std::chrono::steady_clock::now();
            float dt = std::chrono::duration<float>(now_tick - last_tick).count();
            last_tick = now_tick;
            dt = clamp_value(dt, 0.001f, 0.05f);
            elapsed += dt;

            cv::Point2f target_pos = target_func(elapsed);
            target_pos.x = clamp_value(target_pos.x, 5.0f, static_cast<float>(width - 5));
            target_pos.y = clamp_value(target_pos.y, 5.0f, static_cast<float>(height - 5));

            aim.x = step_pid(target_pos.x, aim.x, dt, g, pid_x, max_rate);
            aim.y = step_pid(target_pos.y, aim.y, dt, g, pid_y, max_rate);

            cv::Mat frame(height, width, CV_8UC3, cv::Scalar(30, 30, 30));
            cv::circle(frame, target_pos, 6, cv::Scalar(0, 0, 255), -1);
            cv::drawMarker(frame, aim, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 16, 2);

            std::string title = "Gen " + std::to_string(generation_index) +
                                " | " + name + " " + std::to_string(trial_idx) + "/" +
                                std::to_string(total_trials);
            cv::putText(frame, title, cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX,
                        0.7, cv::Scalar(255, 255, 255), 2);
            cv::imshow("GA Best Tracking", frame);

            int key = cv::waitKey(16);
            if (key == 27 || key == 'q' || key == 'Q') {
                return false;
            }
        }
        return true;
    };

    for (int i = 0; i < kDisplayMethods.random; ++i) {
        if (!run_random_trial(i + 1, kDisplayMethods.random)) {
            cv::destroyWindow("GA Best Tracking");
            return;
        }
    }

    for (int i = 0; i < kDisplayMethods.sine; ++i) {
        auto target_func = [&](float t) {
            return cv::Point2f(center.x + amp_x * std::sin(omega_sine * t),
                               center.y + amp_y * std::sin(omega_sine * t));
        };
        if (!run_motion_trial("Sine", i + 1, kDisplayMethods.sine, target_func)) {
            cv::destroyWindow("GA Best Tracking");
            return;
        }
    }

    for (int i = 0; i < kDisplayMethods.circular; ++i) {
        auto target_func = [&](float t) {
            return cv::Point2f(center.x + amp_x * std::cos(omega_circle * t),
                               center.y + amp_y * std::sin(omega_circle * t));
        };
        if (!run_motion_trial("Circular", i + 1, kDisplayMethods.circular, target_func)) {
            cv::destroyWindow("GA Best Tracking");
            return;
        }
    }

    for (int i = 0; i < kDisplayMethods.lissajous; ++i) {
        auto target_func = [&](float t) {
            return cv::Point2f(center.x + amp_x * std::sin(w_liss_x * t + 0.2f),
                               center.y + amp_y * std::sin(w_liss_y * t));
        };
        if (!run_motion_trial("Lissajous", i + 1, kDisplayMethods.lissajous, target_func)) {
            cv::destroyWindow("GA Best Tracking");
            return;
        }
    }

    cv::destroyWindow("GA Best Tracking");
}
} // namespace

int main() {
    Population population(POPULATION_SIZE);
    population.set_mutation(MUTATION_RATE, MUTATION_SIGMA);
    population.set_elitism(ELITE_COUNT);
    Population::TrackingFitnessConfig config;
    config.targets_per_individual = 5;
    config.seed = 20260206;
    population.set_tracking_fitness(config);

    Genome base_genome;
    population.initialize_random(base_genome);

    std::map<int, Genome> checkpoints;

    std::filesystem::path log_path = std::filesystem::current_path() / "ga_fitness_log.csv";
    std::ofstream fitness_log(log_path);
    if (!fitness_log.is_open()) {
        log_path = std::filesystem::current_path().parent_path() / "ga_fitness_log.csv";
        fitness_log.open(log_path);
    }
    if (!fitness_log.is_open()) {
        log_path = std::filesystem::current_path().parent_path().parent_path() / "ga_fitness_log.csv";
        fitness_log.open(log_path);
    }
    if (fitness_log.is_open()) {
        fitness_log << "generation,best_fitness\n";
        std::cout << "Fitness log: " << log_path << std::endl;
    } else {
        std::cerr << "Failed to open ga_fitness_log.csv for writing. CWD: "
                  << std::filesystem::current_path() << std::endl;
    }

    for (int gen = 0; gen < MAX_GENERATIONS; ++gen) {
        population.evaluate_all();
        const Individual& best = population.best();
        std::cout << "Generation " << population.generation()
                  << " | Best Fitness: " << best.fitness
                  << " | P: " << best.genome.p
                  << ", I: " << best.genome.i
                  << ", D: " << best.genome.d << std::endl;

        if (fitness_log.is_open()) {
            fitness_log << population.generation() << "," << best.fitness << "\n";
        }

        int gen_index = population.generation();
        if (should_display_generation(gen_index)) {
            checkpoints[gen_index] = best.genome;
        }

        population.evolve_next();
    }

    population.evaluate_all();
    const Individual& best = population.best();
    std::cout << "Best PID => P: " << best.genome.p
              << ", I: " << best.genome.i
              << ", D: " << best.genome.d
              << ", Fitness: " << best.fitness
              << std::endl;

    for (int gen : kDisplayGenerations) {
        auto it = checkpoints.find(gen);
        if (it == checkpoints.end()) {
            continue;
        }
        std::cout << "\nPress any key in the window to show generation " << gen << "..." << std::endl;
        run_visualization(it->second, gen);
        std::cout << "Generation " << gen << " demo finished." << std::endl;
    }

    if (fitness_log.is_open()) {
        fitness_log.close();
    }

    return 0;
}
