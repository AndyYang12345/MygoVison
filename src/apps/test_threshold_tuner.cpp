#include "TargetSim/TrainingFrameGenerator.hpp"
#include "TargetTracking/TargetTracker.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// 全局变量
int saturation_thresh = 50;
int value_thresh = 20;
int min_area = 3000;
int max_area = 8000;
bool show_hsv = false;
bool paused = false;

Mat current_frame;
TrainingFrameGenerator* p_generator = nullptr;
TargetTracker tracker;  // 全局实例

// 回调函数 —— 每次滑条改变都会触发
void on_trackbar(int, void*) {
    // ✅ 正确方式：获取配置副本 -> 修改 -> 写回
    TrackerConfig config = tracker.get_config();
    config.saturation_threshold = saturation_thresh;
    config.min_blob_area = min_area;
    config.max_blob_area = max_area;
    tracker.set_config(config);  // 写回
}

void test_threshold_tuner(TrainingFrameGenerator& generator) {
    cout << "\n=== Real-Time Tracking Tuner Mode ===" << endl;
    cout << "Adjust sliders to improve detection!" << endl;
    cout << "Keys:" << endl;
    cout << "  H - Toggle HSV channels view" << endl;
    cout << "  S - Save current recommended settings" << endl;
    cout << "  Space - Pause/Resume" << endl;
    cout << "  ESC - Exit" << endl;

    namedWindow("Threshold Tuner", WINDOW_AUTOSIZE);
    namedWindow("Masks", WINDOW_AUTOSIZE);

    // 创建滑动条（绑定回调）
    createTrackbar("Saturation Thresh", "Threshold Tuner", &saturation_thresh, 255, on_trackbar);
    createTrackbar("Value (Lightness) Thresh", "Threshold Tuner", &value_thresh, 255, on_trackbar);
    createTrackbar("Min Area", "Threshold Tuner", &min_area, 10000, on_trackbar);
    createTrackbar("Max Area", "Threshold Tuner", &max_area, 10000, on_trackbar);

    p_generator = &generator;

    while (true) {
        TargetInfo result;  // ✅ 移到循环内部声明

        if (!paused) {
            auto frame_data = generator.get_next_frame();
            current_frame = frame_data.frame.clone();

            // --- 执行追踪 ---
            result = tracker.process_frame(current_frame);

            // --- 可视化检测结果 ---
            if (result.found) {
                circle(current_frame, result.target_center, 12, Scalar(255, 0, 0), -1);
                putText(current_frame, "TRACKED",
                        Point(static_cast<int>(result.target_center.x + 20),
                              static_cast<int>(result.target_center.y - 10)),
                        FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 0, 0), 1);
            } else {
                putText(current_frame, "NO TARGET DETECTED",
                        Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 1);
            }

            // 显示真实位置（绿色）
            circle(current_frame, frame_data.target_position, 10, Scalar(0, 255, 0), -1);
            putText(current_frame, "GT",
                    Point(static_cast<int>(frame_data.target_position.x + 15),
                          static_cast<int>(frame_data.target_position.y - 15)),
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 1);
        }

        // --- 显示掩码和轮廓 ---
        Mat hsv, s_channel, v_channel;
        cvtColor(current_frame, hsv, COLOR_BGR2HSV);
        vector<Mat> chs;
        split(hsv, chs);
        s_channel = chs[1];
        v_channel = chs[2];

        Mat s_mask, v_mask, combined_mask;
        threshold(s_channel, s_mask, saturation_thresh, 255, THRESH_BINARY);
        threshold(v_channel, v_mask, value_thresh, 255, THRESH_BINARY);
        combined_mask = s_mask | v_mask;

        // 形态学去噪
        Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3,3));
        morphologyEx(combined_mask, combined_mask, MORPH_OPEN, kernel);

        // 查找轮廓
        vector<vector<Point>> contours;
        findContours(combined_mask.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // 绘制预览图
        Mat contour_preview = current_frame.clone();
        int valid_count = 0;
        for (auto& cnt : contours) {
            double area = contourArea(cnt);
            if (area >= min_area && area <= max_area) {
                drawContours(contour_preview, contours,
                            static_cast<int>(&cnt - &contours[0]),
                            Scalar(0, 255, 255), 2);
                Moments m = moments(cnt);
                if (m.m00 != 0) {
                    Point2f center(m.m10 / m.m00, m.m01 / m.m00);
                    circle(contour_preview, center, 4, Scalar(0, 0, 255), -1);
                    putText(contour_preview, to_string(valid_count++),
                           center + Point2f(10, -10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
                }
            }
        }

        // 构建掩码拼接图
        Mat masks_display;
        if (show_hsv) {
            Mat h_mask; threshold(chs[0], h_mask, 10, 255, THRESH_BINARY);
            vector<Mat> stacked = { h_mask, s_mask, v_mask, combined_mask };
            hconcat(stacked, masks_display);
        } else {
            vector<Mat> stacked = { s_mask, v_mask, combined_mask };
            hconcat(stacked, masks_display);
        }
        resize(masks_display, masks_display, Size(masks_display.cols * 1.5, masks_display.rows * 1.5));

        // 添加统计信息
        putText(contour_preview, "Detected: " + string(result.found ? "YES" : "NO"),
               Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, 
               result.found ? Scalar(0, 255, 255) : Scalar(0, 0, 255), 1);
        putText(contour_preview, "Valid Blobs: " + to_string(valid_count),
               Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
        putText(contour_preview, format("S_T=%d, V_T=%d", saturation_thresh, value_thresh),
               Point(10, 90), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
        putText(contour_preview, format("Area: %d~%d", min_area, max_area),
               Point(10, 120), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);

        imshow("Threshold Tuner", contour_preview);
        imshow("Masks", masks_display);

        char key = waitKey(paused ? 0 : 30);
        switch (key) {
            case 'h':
            case 'H':
                show_hsv = !show_hsv;
                cout << "Show HSV components: " << (show_hsv ? "ON" : "OFF") << endl;
                break;

            case 's':
            case 'S': {
                cout << "\n=== Recommended Settings (for TargetTracker) ===" << endl;
                cout << "config_.saturation_threshold = " << saturation_thresh << ";" << endl;
                cout << "config_.min_blob_area = " << min_area << ";" << endl;
                cout << "config_.max_blob_area = " << max_area << ";" << endl;
                cout << "// Use value_thresh=" << value_thresh << " as reference for dark color sensitivity" << endl;
                cout << "✅ Copy these into your tracker config." << endl;
                break;
            }

            case ' ':
                paused = !paused;
                cout << "Pause: " << (paused ? "ON" : "OFF") << endl;
                break;

            case 27:
                goto exit_loop;
        }
    }

exit_loop:
    destroyAllWindows();
    cout << "Tuning ended." << endl;
}

int main() {
    TrainingFrameGenerator generator(450, 450, 60);
    generator.set_training_mode(
        TrainingFrameGenerator::MODE_PENTAGON_ROTATION,
        0.5f
    );

    test_threshold_tuner(generator);
    return 0;
}
