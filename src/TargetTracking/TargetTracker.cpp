#include "TargetTracking/TargetTracker.hpp"
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <limits>

using namespace cv;
using namespace std;

// ============ 构造函数和配置管理 ============

TargetTracker::TargetTracker() 
    : frames_processed_(0), 
      successful_tracks_(0),
      has_previous_target_(false),
      debug_enabled_(true),
      frame_size_(cv::Size(640, 480)) {
    // 默认配置
    config_ = TrackerConfig();
}

void TargetTracker::set_config(const TrackerConfig& config) {
    config_ = config;
}

TrackerConfig TargetTracker::get_config() const {
    return config_;
}

// ============ 主处理流程 ============

TargetInfo TargetTracker::process_frame(const Mat& frame) {
    TargetInfo result;
    result.found = false;
    frames_processed_++;
    
    // 更新帧大小
    frame_size_ = frame.size();
    
    if (frame.empty()) {
        if (config_.print_debug_info) {
            cerr << "[ERROR] Empty frame received!" << endl;
        }
        return result;
    }
    
    if (config_.print_debug_info) {
        cout << "\n=== Processing Frame #" << frames_processed_ << " ===" << endl;
        cout << "Frame size: " << frame.cols << "x" << frame.rows << endl;
    }
    
    // Step 1: 提取所有色块（不进行颜色匹配过滤！）
    Mat debug_mask;
    vector<ColorBlob> blobs = extract_color_blobs(frame, debug_mask);
    
    if (config_.print_debug_info) {
        cout << "Found " << blobs.size() << " color blobs" << endl;
    }
    
    if (blobs.size() < 6) {  // 至少需要6个色块（1个中心 + 5个周围）
        if (config_.print_debug_info) {
            cout << "Insufficient blobs (" << blobs.size() << "), need at least 6" << endl;
        }
        return result;
    }
    
    // Step 2: 找到中心色块
    ColorBlob* center_blob = find_center_blob(blobs);
    if (center_blob == nullptr) {
        if (config_.print_debug_info) {
            cout << "No valid center blob found!" << endl;
        }
        return result;
    }
    
    if (config_.print_debug_info) {
        cout << "Center blob found at (" << center_blob->center.x 
             << ", " << center_blob->center.y << ")" << endl;
        cout << "Center color BGR: [" << center_blob->mean_color_bgr[0] 
             << ", " << center_blob->mean_color_bgr[1] 
             << ", " << center_blob->mean_color_bgr[2] << "]" << endl;
        cout << "Center color HSV: [" << center_blob->mean_color_hsv[0] 
             << ", " << center_blob->mean_color_hsv[1] 
             << ", " << center_blob->mean_color_hsv[2] << "]" << endl;
        cout << "Center is dark: " << (center_blob->is_dark ? "YES" : "NO") << endl;
    }
    
    // Step 3: 找到匹配的目标色块
    ColorBlob* target_blob = find_matching_target(blobs, *center_blob);
    if (target_blob == nullptr) {
        if (config_.print_debug_info) {
            cout << "No matching target blob found!" << endl;
        }
        return result;
    }
    
    // Step 4: 计算结果
    result.found = true;
    result.board_center = center_blob->center;
    result.target_center = target_blob->center;
    
    // 计算距离和角度
    Point2f delta = target_blob->center - center_blob->center;
    result.distance = norm(delta);
    result.angle = atan2(delta.y, delta.x) * 180.0 / CV_PI;
    
    // 更新统计信息
    successful_tracks_++;
    last_target_position_ = target_blob->center;
    last_board_position_ = center_blob->center;
    has_previous_target_ = true;
    
    if (config_.print_debug_info) {
        cout << "SUCCESS: Target found!" << endl;
        cout << "  Target position: (" << target_blob->center.x 
             << ", " << target_blob->center.y << ")" << endl;
        cout << "  Distance: " << result.distance << " pixels" << endl;
        cout << "  Angle: " << result.angle << " degrees" << endl;
    }
    
    // Step 5: 调试显示
    if (config_.show_debug_windows) {
        Mat debug_frame = frame.clone();
        draw_debug_info(debug_frame, blobs, center_blob, target_blob);
        
        // 显示中间结果
        vector<Mat> debug_images;
        debug_images.push_back(debug_frame);
        debug_images.push_back(debug_mask);
        
        Mat combined;
        hconcat(debug_images, combined);
        
        resize(combined, combined, Size(), 0.5, 0.5);
        imshow("Target Tracker Debug", combined);
        waitKey(1);
    }
    
    return result;
}

// ============ 核心处理函数 ============

vector<ColorBlob> TargetTracker::extract_color_blobs(const Mat& frame, Mat& debug_mask) {
    vector<ColorBlob> blobs;
    
    if (config_.print_debug_info) {
        cout << "[DEBUG] extract_color_blobs: Starting..." << endl;
    }
    
    // 1. 预处理：高斯模糊
    Mat blurred;
    GaussianBlur(frame, blurred, 
                 Size(config_.blur_size, config_.blur_size), 
                 config_.blur_sigma);
    
    // 2. 转换到HSV颜色空间
    Mat hsv;
    cvtColor(blurred, hsv, COLOR_BGR2HSV);
    
    // 3. 分离通道
    vector<Mat> channels;
    split(hsv, channels);
    Mat hue = channels[0];
    Mat saturation = channels[1];
    Mat value = channels[2];
    
    // 分析图像整体亮度
    Scalar mean_value = mean(value);
    bool image_is_bright = mean_value[0] > 200;
    
    if (config_.print_debug_info) {
        cout << "[DEBUG] Image mean brightness: " << mean_value[0] << endl;
        cout << "[DEBUG] Image is bright: " << (image_is_bright ? "YES" : "NO") << endl;
    }
    
    // 4. 创建掩码
    Mat sat_mask, val_mask, combined_mask;
    
    // 饱和度掩码
    threshold(saturation, sat_mask, config_.saturation_threshold, 255, THRESH_BINARY);
    
    // 自适应亮度阈值
    int adaptive_val_thresh;
    if (image_is_bright) {
        adaptive_val_thresh = max(config_.value_threshold, 180);
    } else {
        adaptive_val_thresh = config_.value_threshold;
    }
    
    threshold(value, val_mask, adaptive_val_thresh, 255, THRESH_BINARY);
    
    if (config_.print_debug_info) {
        int sat_pixels = countNonZero(sat_mask);
        int val_pixels = countNonZero(val_mask);
        cout << "[DEBUG] Using adaptive brightness threshold: " << adaptive_val_thresh << endl;
        cout << "[DEBUG] Saturation mask: " << sat_pixels << " pixels (" 
             << (sat_pixels * 100.0 / (frame.cols * frame.rows)) << "%)" << endl;
        cout << "[DEBUG] Value mask: " << val_pixels << " pixels (" 
             << (val_pixels * 100.0 / (frame.cols * frame.rows)) << "%)" << endl;
    }
    
    // 5. 组合掩码
    if (image_is_bright) {
        // 亮图像：主要使用饱和度掩码
        combined_mask = sat_mask;
        
        // 特别处理深色：添加低亮度区域
        Mat low_val_mask;
        threshold(value, low_val_mask, 80, 255, THRESH_BINARY_INV);
        Mat dark_sat_mask;
        threshold(saturation, dark_sat_mask, 80, 255, THRESH_BINARY);
        Mat dark_mask = low_val_mask & dark_sat_mask;
        combined_mask = combined_mask | dark_mask;
    } else {
        // 正常图像：同时满足饱和度和亮度条件
        combined_mask = sat_mask & val_mask;
    }
    
    // 6. 形态学操作
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    morphologyEx(combined_mask, combined_mask, MORPH_CLOSE, kernel);
    morphologyEx(combined_mask, combined_mask, MORPH_OPEN, kernel);
    
    // 7. 查找轮廓
    vector<vector<Point>> contours;
    findContours(combined_mask.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    if (config_.print_debug_info) {
        cout << "[DEBUG] Found " << contours.size() << " contours before filtering" << endl;
    }
    
    // 8. 提取色块信息（仅基于几何特征过滤）
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = contourArea(contours[i]);
        
        // 面积过滤
        if (area < config_.min_blob_area || area > config_.max_blob_area) {
            if (config_.print_debug_info && area < config_.min_blob_area) {
                cout << "[DEBUG] Contour " << i << " rejected: area " << area 
                     << " < min " << config_.min_blob_area << endl;
            }
            continue;
        }
        
        ColorBlob blob;
        blob.bounding_rect = boundingRect(contours[i]);
        blob.area = area;
        blob.circularity = calculate_circularity(contours[i]);
        
        // 圆形度过滤
        if (blob.circularity < config_.min_circularity) {
            if (config_.print_debug_info) {
                cout << "[DEBUG] Contour " << i << " rejected: circularity " << blob.circularity 
                     << " < threshold " << config_.min_circularity << endl;
            }
            continue;
        }
        
        // 计算质心
        Moments m = moments(contours[i]);
        if (m.m00 > 0) {
            blob.center = Point2f(m.m10 / m.m00, m.m01 / m.m00);
        } else {
            blob.center = Point2f(blob.bounding_rect.x + blob.bounding_rect.width / 2,
                                 blob.bounding_rect.y + blob.bounding_rect.height / 2);
        }
        
        // 计算平均颜色
        Mat roi = frame(blob.bounding_rect);
        blob.mean_color_bgr = mean(roi);
        blob.mean_color_hsv = bgr_to_hsv(blob.mean_color_bgr);
        
        // 判断是否为深色
        blob.is_dark = is_dark_color(blob.mean_color_bgr, config_.dark_brightness_threshold);
        
        if (config_.print_debug_info) {
            cout << "[DEBUG] Contour " << i << " accepted:" << endl;
            cout << "[DEBUG]   Position: (" << blob.center.x << ", " << blob.center.y << ")" << endl;
            cout << "[DEBUG]   Area: " << blob.area << ", Circularity: " << blob.circularity << endl;
            cout << "[DEBUG]   BGR: (" << blob.mean_color_bgr[0] << ", " 
                 << blob.mean_color_bgr[1] << ", " << blob.mean_color_bgr[2] << ")" << endl;
            cout << "[DEBUG]   HSV: (" << blob.mean_color_hsv[0] << ", " 
                 << blob.mean_color_hsv[1] << ", " << blob.mean_color_hsv[2] << ")" << endl;
            cout << "[DEBUG]   Is dark: " << (blob.is_dark ? "YES" : "NO") << endl;
        }
        
        blobs.push_back(blob);
    }
    
    // 保存调试掩码
    debug_mask = combined_mask;
    
    if (config_.print_debug_info) {
        cout << "[DEBUG] extract_color_blobs: Returning " << blobs.size() << " blobs" << endl;
    }
    
    return blobs;
}

ColorBlob* TargetTracker::find_center_blob(vector<ColorBlob>& blobs) {
    if (blobs.empty()) return nullptr;
    
    ColorBlob* best_center = nullptr;
    float best_score = -1.0f;
    
    // 图像中心
    Point2f image_center(frame_size_.width / 2.0f, frame_size_.height / 2.0f);
    
    for (auto& blob : blobs) {
        // 计算分数：考虑圆形度和位置
        float circularity_score = blob.circularity;
        
        // 位置分数：距离图像中心越近，分数越高
        float dist_to_center = norm(blob.center - image_center);
        float max_dist = norm(Point2f(0, 0) - image_center);
        float position_score = 1.0f - (dist_to_center / max_dist);
        
        // 周围色块数量
        int surrounding_count = 0;
        for (const auto& other : blobs) {
            if (&blob == &other) continue;
            
            float dist = norm(other.center - blob.center);
            if (dist >= config_.min_distance_to_center && 
                dist <= config_.max_distance_to_center) {
                surrounding_count++;
            }
        }
        
        float surround_score = surrounding_count / 5.0f;  // 期望5个
        
        // 综合分数
        float total_score = circularity_score * 0.4f + 
                           position_score * 0.3f + 
                           surround_score * 0.3f;
        
        if (total_score > best_score) {
            best_score = total_score;
            best_center = &blob;
        }
    }
    
    if (config_.print_debug_info && best_center != nullptr) {
        cout << "Center blob score: " << best_score << endl;
        cout << "Surrounding blobs: ";
        for (const auto& blob : blobs) {
            if (&blob == best_center) continue;
            float dist = norm(blob.center - best_center->center);
            if (dist >= config_.min_distance_to_center && 
                dist <= config_.max_distance_to_center) {
                cout << "(" << blob.center.x << "," << blob.center.y << ") ";
            }
        }
        cout << endl;
    }
    
    return best_center;
}

ColorBlob* TargetTracker::find_matching_target(const vector<ColorBlob>& blobs, 
                                              const ColorBlob& center_blob) {
    ColorBlob* best_match = nullptr;
    float best_similarity = -1.0f;
    
    // 计算中心色块是否为暗色
    bool center_is_dark = is_dark_color(center_blob.mean_color_bgr, config_.dark_brightness_threshold);
    
    if (config_.print_debug_info) {
        cout << "Finding match for center at (" << center_blob.center.x 
             << ", " << center_blob.center.y << ")" << endl;
        cout << "Center color BGR: (" << center_blob.mean_color_bgr[0] << ", "
             << center_blob.mean_color_bgr[1] << ", " << center_blob.mean_color_bgr[2] << ")" << endl;
        cout << "Center is dark: " << (center_is_dark ? "YES" : "NO") << endl;
    }
    
    // 方法1：直接取最大值
    for (const auto& blob : blobs) {
        if (&blob == &center_blob) continue;
        if (!is_valid_surrounding_blob(blob, center_blob)) continue;
        
        float similarity = calculate_color_similarity(
            center_blob.mean_color_bgr, 
            blob.mean_color_bgr, 
            center_is_dark
        );
        
        if (config_.print_debug_info) {
            cout << "  Candidate at (" << blob.center.x << "," << blob.center.y 
                 << "): similarity = " << similarity << endl;
        }
        
        if (similarity > best_similarity) {
            best_similarity = similarity;
            best_match = const_cast<ColorBlob*>(&blob);
        }
    }
    
    // 方法2：归一化后取最大值
    // vector<float> similarities;
    // vector<const ColorBlob*> candidates;
    // ... 收集所有相似度
    // vector<float> normalized = normalize_similarities(similarities);
    // int best_idx = 找到最大值索引
    // best_match = candidates[best_idx];
    
    if (best_match != nullptr && config_.print_debug_info) {
        cout << "Selected target at (" << best_match->center.x 
             << ", " << best_match->center.y << ")" << endl;
        cout << "Best similarity: " << best_similarity << endl;
    }
    
    return best_match;
}
// ============ 颜色匹配函数 ============

float TargetTracker::calculate_color_similarity(const Scalar& color1, const Scalar& color2, bool center_is_dark) {
    // 将BGR转换为HSV进行比较
    Scalar hsv1 = bgr_to_hsv(color1);
    Scalar hsv2 = bgr_to_hsv(color2);
    
    if (center_is_dark) {
        // 对于暗色，主要使用BGR距离
        float dist = color_distance_bgr(color1, color2);
        float similarity = 1.0f - min(dist / config_.bgr_distance_threshold, 1.0f);
        return similarity;
    } else {
        // 对于亮色，主要使用HSV色调距离
        float hue_diff = abs(hsv1[0] - hsv2[0]);
        hue_diff = min(hue_diff, 180.0f - hue_diff);  // 色调是环形的
        
        // 同时考虑饱和度和明度
        float sat_diff = abs(hsv1[1] - hsv2[1]);
        float val_diff = abs(hsv1[2] - hsv2[2]);
        
        // 加权计算总差异
        float total_diff = hue_diff * 2.0f + sat_diff * 0.5f + val_diff * 0.3f;
        float similarity = 1.0f - min(total_diff / 200.0f, 1.0f);
        return similarity;
    }
}

// ============ 辅助函数 ============

// 在TargetTracker.cpp中实现
vector<float> TargetTracker::normalize_similarities(const vector<float>& similarities) {
    vector<float> result;
    result.reserve(similarities.size());
    
    if (similarities.empty()) {
        return result;
    }
    
    // 找到最大值和最小值
    float min_val = similarities[0];
    float max_val = similarities[0];
    
    for (float val : similarities) {
        if (val < min_val) min_val = val;
        if (val > max_val) max_val = val;
    }
    
    // 最大最小归一化：映射到0-1范围
    float range = max_val - min_val;
    
    if (range > 0) {
        for (float val : similarities) {
            result.push_back((val - min_val) / range);
        }
    } else {
        // 如果所有值相等，都给0.5（中立值）
        for (size_t i = 0; i < similarities.size(); ++i) {
            result.push_back(0.5f);
        }
    }
    
    return result;
}

double TargetTracker::calculate_circularity(const vector<Point>& contour) {
    double area = contourArea(contour);
    double perimeter = arcLength(contour, true);
    
    if (perimeter == 0) return 0;
    
    double circularity = (4 * CV_PI * area) / (perimeter * perimeter);
    return circularity;
}

bool TargetTracker::is_valid_surrounding_blob(const ColorBlob& blob, 
                                             const ColorBlob& center) {
    // 检查距离
    float distance = norm(blob.center - center.center);
    
    if (distance < config_.min_distance_to_center || 
        distance > config_.max_distance_to_center) {
        return false;
    }
    
    return true;
}

Scalar TargetTracker::bgr_to_hsv(const Scalar& bgr) {
    Mat bgr_mat(1, 1, CV_8UC3, Scalar(bgr[0], bgr[1], bgr[2]));
    Mat hsv_mat;
    cvtColor(bgr_mat, hsv_mat, COLOR_BGR2HSV);
    Vec3b hsv = hsv_mat.at<Vec3b>(0, 0);
    return Scalar(hsv[0], hsv[1], hsv[2]);
}

float TargetTracker::color_distance_bgr(const Scalar& c1, const Scalar& c2) {
    float db = c1[0] - c2[0];
    float dg = c1[1] - c2[1];
    float dr = c1[2] - c2[2];
    return sqrt(db * db + dg * dg + dr * dr);
}

bool TargetTracker::is_dark_color(const Scalar& bgr, int threshold) {
    // 计算亮度: 0.299*R + 0.587*G + 0.114*B
    float brightness = 0.299f * bgr[2] + 0.587f * bgr[1] + 0.114f * bgr[0];
    return brightness < threshold;
}

// ============ 调试功能 ============

void TargetTracker::draw_debug_info(Mat& frame, 
                                   const vector<ColorBlob>& blobs,
                                   const ColorBlob* center,
                                   const ColorBlob* target) {
    // 绘制所有色块
    for (const auto& blob : blobs) {
        // 边界框
        rectangle(frame, blob.bounding_rect, Scalar(255, 0, 0), 2);
        
        // 中心点
        circle(frame, blob.center, 3, Scalar(0, 255, 0), -1);
        
        // 面积和圆形度
        string info = format("A:%.0f C:%.2f", blob.area, blob.circularity);
        putText(frame, info, 
                Point(blob.bounding_rect.x, blob.bounding_rect.y - 5),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
    }
    
    // 绘制中心色块
    if (center != nullptr) {
        circle(frame, center->center, 8, Scalar(0, 255, 255), 3);
        putText(frame, "CENTER", 
                Point(center->center.x + 10, center->center.y),
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
    }
    
    // 绘制目标色块
    if (target != nullptr && center != nullptr) {
        // 目标色块
        circle(frame, target->center, 8, Scalar(0, 0, 255), 3);
        
        // 连接线
        line(frame, center->center, target->center, Scalar(0, 255, 0), 2);
        
        // 距离和角度信息
        float distance = norm(target->center - center->center);
        float angle = atan2(target->center.y - center->center.y,
                           target->center.x - center->center.x) * 180 / CV_PI;
        
        string info = format("Dist: %.1fpx, Angle: %.1f deg", distance, angle);
        putText(frame, info, Point(10, 30), 
                FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255), 2);
        
        // 显示颜色信息
        string color_info = format("Center: (%.0f,%.0f,%.0f)", 
                                  center->mean_color_bgr[0],
                                  center->mean_color_bgr[1],
                                  center->mean_color_bgr[2]);
        putText(frame, color_info, Point(10, 60), 
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 0), 2);
    }
}


void TargetTracker::enable_debug(bool enabled) {
    debug_enabled_ = enabled;
    config_.show_debug_windows = enabled;
    config_.print_debug_info = enabled;
}

void TargetTracker::reset_statistics() {
    frames_processed_ = 0;
    successful_tracks_ = 0;
    has_previous_target_ = false;
}

void TargetTracker::print_statistics() const {
    cout << "\n=== Tracker Statistics ===" << endl;
    cout << "Frames processed: " << frames_processed_ << endl;
    cout << "Successful tracks: " << successful_tracks_ << endl;
    
    if (frames_processed_ > 0) {
        float success_rate = (float)successful_tracks_ / frames_processed_ * 100;
        cout << "Success rate: " << success_rate << "%" << endl;
    }
    
    cout << "=========================" << endl;
}