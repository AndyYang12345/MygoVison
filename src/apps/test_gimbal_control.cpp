#include <opencv2/opencv.hpp>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>

#include "TargetTracking/GimbalControl.hpp"

namespace {
struct SpeedEditor {
    bool active{false};
    std::string buffer;
};

void draw_speed_window(const std::string& win_name,
                       float speed_value,
                       const SpeedEditor& editor,
                       const std::string& hint) {
    cv::Mat img(200, 420, CV_8UC3, cv::Scalar(30, 30, 30));
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << speed_value << " deg/s";

    cv::putText(img, "Speed", {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(220, 220, 220), 2);
    cv::putText(img, oss.str(), {20, 80}, cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(180, 255, 180), 2);

    if (editor.active) {
        std::string input = editor.buffer.empty() ? "(empty)" : editor.buffer;
        cv::putText(img, "Editing: " + input, {20, 120}, cv::FONT_HERSHEY_SIMPLEX,
                    0.6, cv::Scalar(255, 200, 100), 2);
        cv::putText(img, "Enter=apply  Backspace=del", {20, 155},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
    } else {
        cv::putText(img, hint, {20, 125}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(200, 200, 200), 1);
    }

    cv::imshow(win_name, img);
}

bool is_valid_speed_char(int key) {
    return (key >= '0' && key <= '9') || key == '.';
}
} // namespace

int main() {
    GimbalControl gimbal;

    const std::string main_win = "Gimbal Control";
    const std::string pitch_speed_win = "Pitch Speed";
    const std::string yaw_speed_win = "Yaw Speed";

    cv::namedWindow(main_win, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(pitch_speed_win, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(yaw_speed_win, cv::WINDOW_AUTOSIZE);

    int pitch_angle = 90;
    int yaw_angle = 135;

    cv::createTrackbar("Pitch Angle", main_win, &pitch_angle, 180);
    cv::createTrackbar("Yaw Angle", main_win, &yaw_angle, 270);

    float pitch_speed = 60.0f;
    float yaw_speed = 60.0f;

    SpeedEditor pitch_editor;
    SpeedEditor yaw_editor;

    while (true) {
        cv::Mat canvas(480, 720, CV_8UC3, cv::Scalar(20, 20, 20));

        gimbal.set_pitch_angle(static_cast<float>(pitch_angle));
        gimbal.set_yaw_angle(static_cast<float>(yaw_angle));
        gimbal.set_pitch_speed(pitch_speed);
        gimbal.set_yaw_speed(yaw_speed);
        gimbal.get_command();

        std::ostringstream status;
        status << "Pitch: " << pitch_angle << " deg  |  Yaw: " << yaw_angle << " deg";
        cv::putText(canvas, status.str(), {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(220, 220, 220), 2);

        std::ostringstream speed_line;
        speed_line << std::fixed << std::setprecision(1)
                   << "Pitch Speed: " << pitch_speed << " deg/s  |  "
                   << "Yaw Speed: " << yaw_speed << " deg/s";
        cv::putText(canvas, speed_line.str(), {20, 80}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(180, 220, 255), 2);

        const std::string cmd = gimbal.get_command_buffer();
        cv::putText(canvas, "Serial Cmd:", {20, 130}, cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(255, 200, 120), 2);
        cv::putText(canvas, cmd, {20, 170}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(255, 255, 255), 2);

        cv::putText(canvas, "Keys: 1=edit pitch speed, 2=edit yaw speed, q/ESC=quit",
                    {20, 430}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(180, 180, 180), 1);

        cv::imshow(main_win, canvas);

        draw_speed_window(pitch_speed_win, pitch_speed, pitch_editor,
                          "Press 1 to edit pitch speed");
        draw_speed_window(yaw_speed_win, yaw_speed, yaw_editor,
                          "Press 2 to edit yaw speed");

        int key = cv::waitKey(30);
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }

        if (key == '1') {
            pitch_editor.active = true;
            yaw_editor.active = false;
            pitch_editor.buffer.clear();
        } else if (key == '2') {
            yaw_editor.active = true;
            pitch_editor.active = false;
            yaw_editor.buffer.clear();
        } else if (key == 13 || key == 10) {
            if (pitch_editor.active && !pitch_editor.buffer.empty()) {
                pitch_speed = std::max(0.0f, std::stof(pitch_editor.buffer));
                pitch_editor.active = false;
            }
            if (yaw_editor.active && !yaw_editor.buffer.empty()) {
                yaw_speed = std::max(0.0f, std::stof(yaw_editor.buffer));
                yaw_editor.active = false;
            }
        } else if (key == 8 || key == 127) {
            if (pitch_editor.active && !pitch_editor.buffer.empty()) {
                pitch_editor.buffer.pop_back();
            }
            if (yaw_editor.active && !yaw_editor.buffer.empty()) {
                yaw_editor.buffer.pop_back();
            }
        } else if (is_valid_speed_char(key)) {
            if (pitch_editor.active) {
                pitch_editor.buffer.push_back(static_cast<char>(key));
            }
            if (yaw_editor.active) {
                yaw_editor.buffer.push_back(static_cast<char>(key));
            }
        }
    }

    return 0;
}
