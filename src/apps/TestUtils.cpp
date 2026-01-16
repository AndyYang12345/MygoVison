// TestUtils.cpp
#include "TargetTracking/TestUtils.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>

using namespace cv;
using namespace std;

// --- 安全生成诊断图像 ---
void save_debug_result(const cv::Mat& original_image,
                       const TargetInfo& result,
                       const std::vector<ColorBlob>& blobs,
                       int center_idx,
                       size_t best_idx,
                       const std::string& output_image_path,
                       const std::string& log_txt_path) {
    cv::Mat debug_img = original_image.clone();

    // ... 绘制轮廓、中心、目标等 ...

    // === 新增：创建掩码并合成诊断图 ===
    cv::Mat hsv;
    cv::cvtColor(original_image, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> chs;
    cv::split(hsv, chs);

    int sat_thresh = 30; // ← 改成你实际使用的阈值
    cv::Mat s_mask, v_mask, combined_mask;
    cv::threshold(chs[1], s_mask, sat_thresh, 255, cv::THRESH_BINARY);
    cv::threshold(chs[2], v_mask, 20, 255, cv::THRESH_BINARY);
    combined_mask = s_mask | v_mask;

    // 形态学去噪
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
    cv::morphologyEx(combined_mask, combined_mask, cv::MORPH_OPEN, kernel);

    // 拼接三张黑白掩码图
    cv::Mat mask_row;
    cv::hconcat(std::vector<cv::Mat>{s_mask, v_mask, combined_mask}, mask_row);

    // 调整大小以匹配原图宽度
    if (mask_row.cols != debug_img.cols) {
        cv::resize(mask_row, mask_row, debug_img.size());
    }

    // 确保 mask_row 是单通道 → 转换为三通道才能与 RGB 图拼接
    cv::Mat mask_color;
    cv::cvtColor(mask_row, mask_color, cv::COLOR_GRAY2BGR);

    // 垂直拼接：上面是检测图，下面是掩码
    cv::Mat diagnostic;
    std::vector<cv::Mat> stack = {debug_img, mask_color};
    cv::vconcat(stack, diagnostic);

    // 保存诊断图
    cv::imwrite("diagnostic_output.png", diagnostic);
    std::cout << "[DEBUG] Diagnostic image saved to: diagnostic_output.png" << std::endl;
}
