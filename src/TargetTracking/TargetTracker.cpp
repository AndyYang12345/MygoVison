#include "TargetTracker.hpp"
#include <iostream>
#include <algorithm>

using namespace cv;
using namespace std;

// HSV辅助函数实现
float TargetTracker::calculate_color_distance_hsv(const cv::Scalar& hsv1, const cv::Scalar& hsv2) {
    // HSV空间的距离计算
    // Hue分量在0-180之间，需要处理循环特性
    float hue_diff = std::abs(hsv1[0] - hsv2[0]);
    if (hue_diff > 90) {
        hue_diff = 180 - hue_diff; // HSV色调是圆形的
    }
    
    float saturation_diff = std::abs(hsv1[1] - hsv2[1]);
    float value_diff = std::abs(hsv1[2] - hsv2[2]);
    
    // 加权计算，色调权重最高
    return sqrt(2.0f * hue_diff * hue_diff + 
                saturation_diff * saturation_diff + 
                value_diff * value_diff);
}

bool TargetTracker::is_black_color_hsv(const cv::Scalar& hsv_color) {
    // HSV判断黑色：低亮度且低饱和度
    return hsv_color[2] < config_.black_value_threshold && 
           hsv_color[1] < config_.black_saturation_threshold;
}

cv::Scalar TargetTracker::convert_bgr_to_hsv(const cv::Scalar& bgr_color) {
    // 将BGR颜色转换为HSV
    cv::Mat bgr_mat(1, 1, CV_8UC3);
    bgr_mat.at<cv::Vec3b>(0, 0) = cv::Vec3b(
        static_cast<uchar>(bgr_color[0]),
        static_cast<uchar>(bgr_color[1]),
        static_cast<uchar>(bgr_color[2])
    );
    
    cv::Mat hsv_mat;
    cv::cvtColor(bgr_mat, hsv_mat, cv::COLOR_BGR2HSV);
    
    cv::Vec3b hsv = hsv_mat.at<cv::Vec3b>(0, 0);
    return cv::Scalar(hsv[0], hsv[1], hsv[2]);
}

float TargetTracker::calculate_surround_score(const std::vector<ColorBlob>& blobs, size_t center_idx) {
    if (center_idx >= blobs.size()) return 0.0f;
    
    const auto& center_blob = blobs[center_idx];
    int surrounding_count = 0;
    
    for (size_t i = 0; i < blobs.size(); i++) {
        if (i == center_idx) continue;
        
        const auto& blob = blobs[i];
        float distance = cv::norm(blob.center - center_blob.center);
        
        if (distance < config_.max_distance_to_center && 
            distance > config_.min_distance_to_center) {
            surrounding_count++;
        }
    }
    
    return surrounding_count / 5.0f; // 归一化到0-1
}

void TargetTracker::assign_color_labels_by_hsv(std::vector<ColorBlob>& blobs) {
    if (blobs.empty()) return;
    
    // 1. 为每个色块计算HSV颜色
    for (auto& blob : blobs) {
        blob.color_hsv = convert_bgr_to_hsv(blob.color_bgr);
        blob.is_black = is_black_color_hsv(blob.color_hsv);
    }
    
    // 2. 找到中心色块（基于圆形度和被围绕程度）
    size_t center_idx = 0;
    float best_center_score = -1.0f;
    
    for (size_t i = 0; i < blobs.size(); i++) {
        // 跳过太暗的色块（可能是黑色）
        if (blobs[i].color_hsv[2] < config_.value_min_threshold) {
            continue;
        }
        
        float circularity_score = std::min(1.0f, blobs[i].circularity / 0.8f);
        float surround_score = calculate_surround_score(blobs, i);
        
        float score = circularity_score * 0.7f + surround_score * 0.3f;
        
        if (score > best_center_score) {
            best_center_score = score;
            center_idx = i;
        }
    }
    
    // 如果找不到合适的中心，使用原来的策略
    if (best_center_score < 0) {
        for (size_t i = 0; i < blobs.size(); i++) {
            float score = blobs[i].circularity;
            if (score > best_center_score) {
                best_center_score = score;
                center_idx = i;
            }
        }
    }
    
    cv::Scalar center_hsv = blobs[center_idx].color_hsv;
    blobs[center_idx].color_label = 0; // 中心标签0
    
    // 3. 计算与中心的HSV距离并分类
    vector<pair<float, size_t>> hsv_distances; // (距离, 索引)
    
    for (size_t i = 0; i < blobs.size(); i++) {
        if (i == center_idx) continue;
        
        float dist = calculate_color_distance_hsv(center_hsv, blobs[i].color_hsv);
        hsv_distances.push_back({dist, i});
    }
    
    // 按HSV距离排序
    sort(hsv_distances.begin(), hsv_distances.end());
    
    // 4. 智能分配标签（考虑黑色干扰）
    // 策略：与中心色调相近的标记为相同颜色（标签0），其他标记为不同颜色
    
    // 收集可能的同色块（考虑色调相似性）
    vector<size_t> same_color_indices;
    
    for (const auto& dist_pair : hsv_distances) {
        size_t idx = dist_pair.second;
        float hue_diff = abs(center_hsv[0] - blobs[idx].color_hsv[0]);
        if (hue_diff > 90) hue_diff = 180 - hue_diff; // 处理循环
        
        // 如果色调相近，并且不是黑色
        if (hue_diff < config_.hue_similarity_threshold && !blobs[idx].is_black) {
            same_color_indices.push_back(idx);
        }
    }
    
    // 分配标签：
    // - 中心：标签0
    // - 与中心同色的外围块：标签0
    // - 其他：标签1,2,3...
    // - 黑色块：特殊标签（或者排除）
    
    int next_label = 1;
    
    // 首先，所有同色块（包括可能的目标色块）都标记为0
    for (size_t idx : same_color_indices) {
        blobs[idx].color_label = 0;
    }
    
    // 其他块分配不同标签
    for (size_t i = 0; i < blobs.size(); i++) {
        if (i == center_idx) continue;
        
        // 检查是否已经在同色列表中
        bool is_same_color = false;
        for (size_t same_idx : same_color_indices) {
            if (i == same_idx) {
                is_same_color = true;
                break;
            }
        }
        
        if (is_same_color) {
            continue; // 已经分配为0
        }
        
        // 如果是黑色，可以特殊处理（比如分配特殊标签或排除）
        if (blobs[i].is_black) {
            blobs[i].color_label = 99; // 特殊标签表示黑色
        } else {
            blobs[i].color_label = next_label++;
        }
    }
}

target_info TargetTracker::process_frame(const cv::Mat& frame) {
    target_info result;
    result.found = false;
    
    total_frames_++;
    
    // 步骤1: 图像预处理
    cv::Mat processed;
    cv::GaussianBlur(frame, processed, cv::Size(3, 3), 0);
    
    // 步骤2: HSV转换和饱和度筛选
    cv::Mat hsv;
    cv::cvtColor(processed, hsv, cv::COLOR_BGR2HSV);
    
    std::vector<cv::Mat> hsv_channels;
    cv::split(hsv, hsv_channels);
    cv::Mat saturation_mask;
    
    cv::threshold(hsv_channels[1], saturation_mask, 
                 config_.saturation_threshold, 255, cv::THRESH_BINARY);
    
    // 步骤3: 查找所有轮廓
    std::vector<std::vector<cv::Point>> all_contours;
    cv::findContours(saturation_mask.clone(), all_contours, 
                    cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // 步骤4: 筛选并分析候选色块
    std::vector<ColorBlob> candidate_blobs;
    
    for (size_t i = 0; i < all_contours.size(); i++) {
        double area = cv::contourArea(all_contours[i]);
        
        if (area < config_.min_blob_area || area > config_.max_blob_area) {
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
        
        // 提取该区域的代表颜色（BGR）
        cv::Mat roi = frame(blob.bounding_rect);
        cv::Scalar mean_color = cv::mean(roi);
        blob.color_bgr = mean_color;
        
        // 计算宽高比（用于矩形识别）
        cv::RotatedRect rotated_rect = cv::minAreaRect(all_contours[i]);
        float width = rotated_rect.size.width;
        float height = rotated_rect.size.height;
        blob.aspect_ratio = (width > height) ? width / height : height / width;
        
        candidate_blobs.push_back(blob);
    }
    
    // 如果没有至少6个色块，直接返回
    if (candidate_blobs.size() < 6) {
        return result;
    }
    
    // 步骤5: 使用HSV色彩空间分配颜色标签
    assign_color_labels_by_hsv(candidate_blobs);
    
    // 步骤6: 识别中心色块
    cv::Point2f putative_center;
    int center_label = -1;
    float best_center_score = -1.0f;
    
    for (size_t idx = 0; idx < candidate_blobs.size(); idx++) {
        const auto& blob = candidate_blobs[idx];
        
        // 跳过黑色或太暗的色块
        if (blob.is_black || blob.color_hsv[2] < config_.value_min_threshold) {
            continue;
        }
        
        if (blob.circularity < config_.circularity_threshold) {
            continue;
        }
        
        // 计算被围绕程度
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
        
        if (score > best_center_score) {
            best_center_score = score;
            putative_center = blob.center;
            center_label = blob.color_label;
        }
    }
    
    if (center_label == -1) {
        return result;
    }
    
    // 步骤7: 寻找与中心同色的外围色块
    cv::Point2f target_square_center;
    float best_square_score = -1.0f;
    
    for (size_t idx = 0; idx < candidate_blobs.size(); idx++) {
        const auto& blob = candidate_blobs[idx];
        
        // 排除中心自身
        float dist_to_center = cv::norm(blob.center - putative_center);
        if (dist_to_center < 10) {
            continue;
        }
        
        // 必须与中心颜色标签相同
        if (blob.color_label != center_label) {
            continue;
        }
        
        // 跳过黑色或太暗的色块（额外检查）
        if (blob.is_black) {
            continue;
        }
        
        // 应该是矩形（外围方块），检查宽高比
        if (blob.aspect_ratio > config_.max_aspect_ratio) {
            continue;
        }
        
        // 计算指向中心的方向一致性
        cv::Vec2f to_center(putative_center.x - blob.center.x,
                           putative_center.y - blob.center.y);
        float to_center_norm = cv::norm(to_center);
        to_center /= to_center_norm;
        
        cv::RotatedRect rotated_rect = cv::minAreaRect(blob.contour);
        float angle_rad = rotated_rect.angle * CV_PI / 180;
        cv::Vec2f rect_direction(cos(angle_rad), sin(angle_rad));
        
        float direction_score = std::abs(to_center.dot(rect_direction));
        
        // 距离评分
        float distance_score = 1.0f - std::abs(to_center_norm - config_.expected_radius) / config_.expected_radius;
        distance_score = std::max(0.0f, distance_score);
        
        // 宽高比评分（更接近正方形的得分更高）
        float aspect_score = 1.0f / blob.aspect_ratio;
        
        // 综合评分
        float score = direction_score * config_.direction_weight + 
                     distance_score * config_.distance_weight + 
                     aspect_score * config_.aspect_ratio_weight;
        
        if (score > best_square_score) {
            best_square_score = score;
            target_square_center = blob.center;
        }
    }
    
    // 步骤8: 验证并输出结果
    if (best_square_score > config_.match_threshold) {
        result.found = true;
        result.target_center = target_square_center;
        result.board_center = putative_center;
        
        // 计算相对位置
        cv::Point2f relative = target_square_center - putative_center;
        result.distance = cv::norm(relative);
        result.angle = std::atan2(relative.y, relative.x) * 180 / CV_PI;
        result.center_color_label = center_label;
        
        successful_detections_++;
    }
    
    return result;
}