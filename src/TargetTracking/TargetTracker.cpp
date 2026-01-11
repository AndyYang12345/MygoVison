#include "TargetTracker.hpp"
#include <iostream>
#include <algorithm>

using namespace cv;
using namespace std;

target_info TargetTracker::process_frame(const cv::Mat& frame) {
    // cout << "\n========== TargetTracker::process_frame ==========" << endl;
    target_info result;
    result.found = false;
    
    // 步骤1: 图像预处理
    // cout << "[1] Image preprocessing..." << endl;
    cv::Mat processed;
    cv::GaussianBlur(frame, processed, cv::Size(3, 3), 0);
    // cout << "  Input frame: " << frame.cols << "x" << frame.rows << endl;
    
    // 步骤2: 颜色空间转换和饱和度筛选
    // cout << "[2] HSV conversion and saturation filtering..." << endl;
    cv::Mat hsv;
    cv::cvtColor(processed, hsv, cv::COLOR_BGR2HSV);
    
    std::vector<cv::Mat> hsv_channels;
    cv::split(hsv, hsv_channels);
    cv::Mat saturation_mask;
    
    cv::threshold(hsv_channels[1], saturation_mask, 
                 config_.saturation_threshold, 255, cv::THRESH_BINARY);
    
    // int saturation_white_pixels = cv::countNonZero(saturation_mask);
    // float saturation_ratio = 100.0f * saturation_white_pixels / (frame.rows * frame.cols);
    // cout << "  Saturation mask: " << saturation_white_pixels 
    //      << " white pixels (" << saturation_ratio << "%)" << endl;
    
    // 步骤3: 查找所有轮廓
    // cout << "[3] Finding contours..." << endl;
    std::vector<std::vector<cv::Point>> all_contours;
    cv::findContours(saturation_mask.clone(), all_contours, 
                    cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // cout << "  Found " << all_contours.size() << " initial contours" << endl;
    
    // 步骤4: 筛选并分析候选色块
    // cout << "[4] Filtering candidate blobs..." << endl;
    std::vector<ColorBlob> candidate_blobs;
    
    for (size_t i = 0; i < all_contours.size(); i++) {
        double area = cv::contourArea(all_contours[i]);
        
        if (area < config_.min_blob_area) {
            // cout << "  Contour " << i << ": area=" << area 
            //      << " -> REJECTED (too small)" << endl;
            continue;
        }
        if (area > config_.max_blob_area) {
            // cout << "  Contour " << i << ": area=" << area 
            //      << " -> REJECTED (too large)" << endl;
            continue;
        }
        
        ColorBlob blob;
        blob.contour = all_contours[i];
        blob.bounding_rect = cv::boundingRect(all_contours[i]);
        
        // 计算中心点
        cv::Moments m = cv::moments(all_contours[i]);
        if (m.m00 != 0) {
            blob.center = cv::Point2f(m.m10 / m.m00, m.m01 / m.m00);
        }
        
        // 计算形状特征
        double perimeter = cv::arcLength(all_contours[i], true);
        blob.circularity = (perimeter == 0) ? 0 : (4 * CV_PI * area) / (perimeter * perimeter);
        
        // 提取该区域的代表颜色
        cv::Mat roi = frame(blob.bounding_rect);
        cv::Scalar mean_color = cv::mean(roi);
        blob.color_bgr = mean_color;
        
        candidate_blobs.push_back(blob);
        
        // cout << "  Contour " << i << ": area=" << area 
        //      << " -> ACCEPTED, center=(" << blob.center.x << ", " << blob.center.y
        //      << "), circularity=" << blob.circularity << endl;
    }
    
    // cout << "  Total accepted blobs: " << candidate_blobs.size() << endl;
    
    // 如果没有至少6个色块，直接返回
    if (candidate_blobs.size() < 6) {
        // cout << "[ERROR] Not enough blobs (" << candidate_blobs.size() << " < 6)" << endl;
        return result;
    }
    
    // 步骤5: 颜色标签分配（基于颜色距离）
    // cout << "[5] Assigning color labels based on similarity..." << endl;
    
    // 使用颜色距离分配标签
    assign_color_labels_by_distance(candidate_blobs, config_.color_similarity_threshold);
    
    // 显示标签分配结果
    // cout << "  Color label assignment results:" << endl;
    map<int, vector<int>> label_to_blobs;
    for (size_t i = 0; i < candidate_blobs.size(); i++) {
        int label = candidate_blobs[i].color_label;
        label_to_blobs[label].push_back(i);
    }
    
    for (const auto& pair : label_to_blobs) {
        // int label = pair.first;
        const vector<int>& blobs = pair.second;
        // cout << "    Label " << label << ": " << blobs.size() << " blobs [";
        for (size_t j = 0; j < blobs.size(); j++) {
            // cout << blobs[j];
            // if (j < blobs.size() - 1) cout << ", ";
        }
        // cout << "]" << endl;
    }
    
    // 步骤6: 识别中心色块
    // cout << "[6] Identifying center blob..." << endl;
    int center_label = -1;
    cv::Point2f putative_center;
    float best_center_score = -1.0f;
    // int best_center_index = -1;
    
    for (size_t idx = 0; idx < candidate_blobs.size(); idx++) {
        const auto& blob = candidate_blobs[idx];
        
        // cout << "  Checking blob " << idx << ": circularity=" << blob.circularity;
        
        if (blob.circularity < config_.circularity_threshold) {
            cout << " -> REJECTED (circularity too low)" << endl;
            continue;
        }
        
        // 计算被其他色块围绕的程度
        int surrounding_count = 0;
        for (size_t j = 0; j < candidate_blobs.size(); j++) {
            if (idx == j) continue;
            
            const auto& other = candidate_blobs[j];
            float distance = cv::norm(other.center - blob.center);
            
            if (distance < config_.max_distance_to_center && 
                distance > config_.min_distance_to_center) {
                surrounding_count++;
            }
        }
        
        float score = blob.circularity * config_.circularity_weight + 
                     (surrounding_count / 5.0f) * config_.surround_weight;
        
        // cout << ", surrounding=" << surrounding_count 
        //      << "/5, score=" << score << endl;
        
        if (score > best_center_score) {
            best_center_score = score;
            putative_center = blob.center;
            center_label = blob.color_label;
            // best_center_index = idx;
        }
    }
    
    if (center_label == -1) {
        // cout << "[ERROR] No center blob found!" << endl;
        return result;
    }
    
    // cout << "  Selected center: blob " << best_center_index 
    //      << " at (" << putative_center.x << ", " << putative_center.y 
    //      << "), label=" << center_label << ", score=" << best_center_score << endl;
    
    // 步骤7: 寻找与中心同色的外围色块
    // cout << "[7] Finding matching outer square (color label=" << center_label << ")..." << endl;
    cv::Point2f target_square_center;
    float best_square_score = -1.0f;
    // int best_square_index = -1;
    
    for (size_t idx = 0; idx < candidate_blobs.size(); idx++) {
        const auto& blob = candidate_blobs[idx];
        
        // 排除中心自身
        float dist_to_center = cv::norm(blob.center - putative_center);
        if (dist_to_center < 10) {
            // cout << "  Blob " << idx << ": too close to center -> SKIP" << endl;
            continue;
        }
        
        // 必须与中心颜色相同
        if (blob.color_label != center_label) {
            // cout << "  Blob " << idx << ": wrong color (label=" << blob.color_label 
            //      << " != " << center_label << ") -> SKIP" << endl;
            continue;
        }
        
        // cout << "  Blob " << idx << ": correct color, checking shape..." << endl;
        
        // 应该是矩形（外围方块）
        cv::RotatedRect rotated_rect = cv::minAreaRect(blob.contour);
        float width = rotated_rect.size.width;
        float height = rotated_rect.size.height;
        float aspect_ratio = (width > height) ? width / height : height / width;
        
        // cout << "    Aspect ratio: " << aspect_ratio;
        
        if (aspect_ratio > config_.max_aspect_ratio) {
            // cout << " -> REJECTED (aspect ratio too high)" << endl;
            continue;
        }
        
        // 计算指向中心的方向一致性
        cv::Vec2f to_center(putative_center.x - blob.center.x,
                           putative_center.y - blob.center.y);
        float to_center_norm = cv::norm(to_center);
        to_center /= to_center_norm;
        
        float angle_rad = rotated_rect.angle * CV_PI / 180;
        cv::Vec2f rect_direction(cos(angle_rad), sin(angle_rad));
        
        float direction_score = std::abs(to_center.dot(rect_direction));
        // cout << ", direction score: " << direction_score;
        
        // 距离评分
        float distance_score = 1.0f - std::abs(to_center_norm - config_.expected_radius) / config_.expected_radius;
        distance_score = std::max(0.0f, distance_score);
        // cout << ", distance: " << to_center_norm << "px (score: " << distance_score << ")";
        
        // 宽高比评分
        float aspect_score = 1.0f / aspect_ratio;
        // cout << ", aspect score: " << aspect_score;
        
        // 综合评分
        float score = direction_score * config_.direction_weight + 
                     distance_score * config_.distance_weight + 
                     aspect_score * config_.aspect_ratio_weight;
        
        // cout << ", TOTAL SCORE: " << score << endl;
        
        if (score > best_square_score) {
            best_square_score = score;
            target_square_center = blob.center;
            // best_square_index = idx;
        }
    }
    
    // 步骤8: 验证并输出结果
    // cout << "[8] Final validation..." << endl;
    if (best_square_score > config_.match_threshold) {
        result.found = true;
        result.target_center = target_square_center;
        result.board_center = putative_center;
        
        // 计算相对位置
        cv::Point2f relative = target_square_center - putative_center;
        result.distance = cv::norm(relative);
        result.angle = std::atan2(relative.y, relative.x) * 180 / CV_PI;
        result.center_color_label = center_label;
        
        // cout << "  TARGET FOUND! Blob " << best_square_index 
        //      << " at (" << target_square_center.x << ", " << target_square_center.y << ")" << endl;
        // cout << "  Distance to center: " << result.distance << "px" << endl;
        // cout << "  Angle: " << result.angle << " degrees" << endl;
        // cout << "  Match score: " << best_square_score << endl;
    } else {
        // cout << "[ERROR] No valid outer square found!" << endl;
        // cout << "  Best score: " << best_square_score 
        //       << " < threshold: " << config_.match_threshold << endl;
    }

    // cout << "========== End of process_frame ==========" << endl << endl;
    return result;
}

void TargetTracker::assign_color_labels_by_distance(std::vector<ColorBlob>& blobs, float similarity_threshold) {
    int next_label = 0;
    vector<int> labels(blobs.size(), -1); // -1表示未分配
    
    for (size_t i = 0; i < blobs.size(); i++) {
        if (labels[i] != -1) continue; // 已分配标签
        
        // 为新颜色创建标签
        labels[i] = next_label;
        
        // 寻找所有与当前色块颜色相似的色块
        for (size_t j = i + 1; j < blobs.size(); j++) {
            if (labels[j] == -1) {
                float distance = calculate_color_distance(blobs[i].color_bgr, blobs[j].color_bgr);
                if (distance < similarity_threshold) {
                    labels[j] = next_label; // 分配到同一标签
                }
            }
        }
        
        next_label++;
    }
    
    // 将标签分配回blobs
    for (size_t i = 0; i < blobs.size(); i++) {
        blobs[i].color_label = labels[i];
    }
}