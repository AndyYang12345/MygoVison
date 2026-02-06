#include "TargetSim/Target3DGenerator.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

int main() {
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

    cv::namedWindow("Target3D", cv::WINDOW_AUTOSIZE);

    std::cout << "Press Q or ESC to quit." << std::endl;

    while (true) {
        auto result = generator.generate_projected_frame();
        cv::Mat frame = result.projected_frame.clone();

        if (!result.projected_positions.empty()) {
            cv::circle(frame, result.projected_positions[0], 5, cv::Scalar(0, 0, 255), -1);
        }

        cv::imshow("Target3D", frame);

        int key = cv::waitKey(16);
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }
    }

    return 0;
}
