// TestUtils.hpp
#pragma once
#include "TargetTracking/TargetTracker.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

void save_debug_result(const cv::Mat& original_image,
                       const TargetInfo& result,
                       const std::vector<ColorBlob>& blobs,
                       int center_idx,
                       size_t best_idx,
                       const std::string& output_image_path = "output_detected.jpg",
                       const std::string& log_txt_path = "detection_log.txt");