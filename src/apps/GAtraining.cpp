#include "TargetTracking/GeneticAlgorithm.hpp"
#include "TargetSim/Target3DGenerator.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <vector>

#define POPULATION_SIZE 50
#define MUTATION_RATE 0.3f
#define MUTATION_SIGMA 0.2f
#define MAX_GENERATIONS 100
#define ELITE_COUNT 2

// 修改这里来指定需要展示的代数
static const std::vector<int> kDisplayGenerations = {1, 50, 75, 99};

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

    cv::Point2f aim(static_cast<float>(width) * 0.5f, static_cast<float>(height) * 0.5f);
    PidState pid_x;
    PidState pid_y;
    const float max_rate = 600.0f; // pixels/s
    const float settle_threshold = 5.0f;
    const float settle_hold = 0.2f;
    const float target_interval = 0.6f;
    generator.set_random_interval(target_interval);
    generator.generator().pause();

    int targets_done = 0;
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
                    targets_done += 1;
                    settle_timer = 0.0f;
                    tracking_active = false;
                    pid_x = {};
                    pid_y = {};
                    request_new_target = true;
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
                            " | target " + std::to_string(targets_done + 1) + "/5";
        cv::putText(frame, title, cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX,
                    0.7, cv::Scalar(255, 255, 255), 2);
        cv::imshow("GA Best Tracking", frame);

        int key = cv::waitKey(16);
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }
        if (targets_done >= 5) {
            break;
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

    for (int gen = 0; gen < MAX_GENERATIONS; ++gen) {
        population.evaluate_all();
        const Individual& best = population.best();
        std::cout << "Generation " << population.generation()
                  << " | Best Fitness: " << best.fitness
                  << " | P: " << best.genome.p
                  << ", I: " << best.genome.i
                  << ", D: " << best.genome.d << std::endl;

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

    return 0;
}