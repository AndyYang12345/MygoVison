#include "maix_target_tracker.hpp"
#include "TargetSim/TrainingFrameGenerator.hpp"
#include "TargetSim/PerformanceMonitor.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

int main(int argc, char **argv)
{
    bool show = false;
    bool show_ground_truth = true;
    bool show_markers = true;
    bool show_processing_time = true;
    bool use_maix = false;
    int frames = 300;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--show") {
            show = true;
        } else if (arg == "--maix") {
            use_maix = true;
        } else if (arg == "--frames" && i + 1 < argc) {
            frames = std::stoi(argv[++i]);
        }
    }

    TrainingFrameGenerator generator(640, 640, 30.0f);
    auto angular_velocity_func = [](float t) -> float {
        (void)t;
        return 1.0f;
    };
    generator.set_training_mode(TrainingFrameGenerator::MODE_PENTAGON_ROTATION, 1.0f, 0.0f, angular_velocity_func);

    maix::vision::TargetTracker tracker;
    maix::vision::TrackerConfig config = tracker.get_config();
    if (use_maix) {
        std::cout << "[WARN] 当前构建为 OpenCV-only，--maix 将被忽略。" << std::endl;
        config.use_maix_find_blobs = false;
    }
    config.print_debug_info = false;
    config.show_debug_windows = false;
    tracker.set_config(config);

    int found_count = 0;
    int total_count = 0;
    double total_error = 0.0;
    double total_processing_time = 0.0;
    double avg_processing_time = 0.0;
    PerformanceMonitor perf_monitor;

    if (show) {
        cv::namedWindow("tracker_sim", cv::WINDOW_AUTOSIZE);
    }

    while (true) {
        auto start_time = std::chrono::high_resolution_clock::now();
        auto frame_data = generator.get_next_frame();
        cv::Mat frame = frame_data.frame;
        maix::vision::TargetInfo result = tracker.process_frame(frame);
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        float processing_time_ms = duration.count() / 1000.0f;
        total_processing_time += processing_time_ms;
        avg_processing_time = total_processing_time / (total_count + 1);

        total_count++;
        double frame_error = -1.0;
        if (result.found) {
            found_count++;
            frame_error = cv::norm(result.target_center - frame_data.target_position);
            total_error += frame_error;
        }

        if (show) {
            cv::Mat display = frame.clone();

            if (show_markers && result.found) {
                cv::circle(display, result.target_center, 8, cv::Scalar(0, 255, 255), -1);
                cv::circle(display, result.board_center, 6, cv::Scalar(0, 255, 0), -1);
                cv::line(display, result.board_center, result.target_center, cv::Scalar(0, 255, 0), 2);
            }

            if (show_ground_truth) {
                cv::circle(display, frame_data.target_position, 8, cv::Scalar(255, 0, 0), 2);
                if (result.found) {
                    cv::line(display, frame_data.target_position, result.target_center,
                             cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
                }
            }

            float fps = perf_monitor.tick();
            float accuracy = total_count > 0 ? (100.0f * found_count / total_count) : 0.0f;

            cv::Rect info_rect(5, 5, 260, 120);
            cv::Mat overlay = display.clone();
            cv::rectangle(overlay, info_rect, cv::Scalar(0, 0, 0), -1);
            cv::addWeighted(overlay, 0.6, display, 0.4, 0, display);

            int y = 25;
            int line = 20;
            cv::putText(display, "FPS: " + std::to_string((int)fps),
                        cv::Point(15, y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 255, 0), 1);
            y += line;
            cv::putText(display, "Accuracy: " + std::to_string(accuracy).substr(0, 5) + "%",
                        cv::Point(15, y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(255, 255, 255), 1);
            y += line;
            std::string err_text = (frame_error >= 0.0) ?
                ("Error: " + std::to_string(frame_error).substr(0, 5) + "px") : "Error: N/A";
            cv::putText(display, err_text,
                        cv::Point(15, y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(255, 255, 255), 1);
            y += line;
            cv::putText(display, "Frames: " + std::to_string(total_count) +
                        " (" + std::to_string(found_count) + ")",
                        cv::Point(15, y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(255, 255, 255), 1);
            y += line;
            if (show_processing_time) {
                cv::putText(display, "Process: " + std::to_string(processing_time_ms).substr(0, 5) + "ms",
                            cv::Point(15, y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            cv::Scalar(255, 255, 255), 1);
                y += line;
                cv::putText(display, "Avg: " + std::to_string(avg_processing_time).substr(0, 5) + "ms",
                            cv::Point(15, y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            cv::Scalar(255, 255, 255), 1);
            }

            cv::imshow("tracker_sim", display);
            int key = cv::waitKey(30);
            if (key == 27 || key == 'q') {
                break;
            } else if (key == 'c' || key == 'C') {
                generator.regenerate_pentagon(generator.get_target_sim_center());
                total_count = 0;
                found_count = 0;
                total_error = 0.0;
                total_processing_time = 0.0;
                avg_processing_time = 0.0;
            } else if (key == 'g' || key == 'G') {
                show_ground_truth = !show_ground_truth;
            } else if (key == 't' || key == 'T') {
                show_markers = !show_markers;
            } else if (key == 'p' || key == 'P') {
                show_processing_time = !show_processing_time;
            }
        } else if (total_count >= frames) {
            break;
        }
    }

    if (show) {
        cv::destroyAllWindows();
    }

    double avg_error = (found_count > 0) ? (total_error / found_count) : 0.0;

    std::cout << "Total frames: " << total_count << std::endl;
    std::cout << "Found frames: " << found_count << std::endl;
    std::cout << "Success rate: " << (total_count > 0 ? (100.0 * found_count / total_count) : 0.0) << "%" << std::endl;
    std::cout << "Avg error (pixels): " << avg_error << std::endl;

    return 0;
}
