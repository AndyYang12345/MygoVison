#include "TargetSim/PentagonSimulator.hpp"
#include <cmath>
#include <random>
#include <opencv2/imgproc.hpp>

namespace {

cv::Point3f mat_vec_mul(const cv::Mat& m, const cv::Point3f& v) {
    return cv::Point3f(
        m.at<float>(0, 0) * v.x + m.at<float>(0, 1) * v.y + m.at<float>(0, 2) * v.z,
        m.at<float>(1, 0) * v.x + m.at<float>(1, 1) * v.y + m.at<float>(1, 2) * v.z,
        m.at<float>(2, 0) * v.x + m.at<float>(2, 1) * v.y + m.at<float>(2, 2) * v.z
    );
}

float dot3(const cv::Point3f& a, const cv::Point3f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

cv::Point3f normalize3(const cv::Point3f& v) {
    const float n = std::sqrt(dot3(v, v));
    if (n < 1e-6f) {
        return cv::Point3f(0.0f, 0.0f, 0.0f);
    }
    return cv::Point3f(v.x / n, v.y / n, v.z / n);
}

bool intersect_ray_with_plane(const cv::Point3f& ray_origin,
                              const cv::Point3f& ray_dir,
                              const cv::Point3f& plane_normal,
                              float plane_distance,
                              cv::Point3f& hit_point) {
    const cv::Point3f n = normalize3(plane_normal);
    const float denom = dot3(n, ray_dir);
    if (std::abs(denom) < 1e-6f) {
        return false;
    }

    const float t = (plane_distance - dot3(n, ray_origin)) / denom;
    if (t <= 0.0f) {
        return false;
    }

    hit_point = cv::Point3f(
        ray_origin.x + ray_dir.x * t,
        ray_origin.y + ray_dir.y * t,
        ray_origin.z + ray_dir.z * t
    );
    return true;
}

} // namespace

PentagonSimulator::PentagonSimulator()
    : config_(), camera_pitch_(0.35f), camera_yaw_(0.0f), paused_(false), frame_count_(0) {
    initialize(config_);
}

PentagonSimulator::PentagonSimulator(const CameraConfig& config)
    : config_(config), camera_pitch_(config.pitch), camera_yaw_(config.yaw), paused_(false), frame_count_(0) {
    initialize(config);
}

PentagonSimulator::~PentagonSimulator() = default;

void PentagonSimulator::initialize(const CameraConfig& config) {
    config_ = config;
    camera_pitch_ = config.pitch;
    camera_yaw_ = config.yaw;
    frame_count_ = 0;
    paused_ = false;
    has_last_target_ = false;
    has_last_laser_ = false;

    // 创建靶子生成器
    generator_ = std::make_unique<TrainingFrameGenerator>(config_.width, config_.height, config_.fps);
    auto velocity_func = energy_mechanism_velocity_generator();
    generator_->set_training_mode(TrainingFrameGenerator::MODE_PENTAGON_ROTATION, 1.0f, 1.0f, velocity_func);

    // 源相机（生成靶子的相机 - 俯视）
    src_camera_ = std::make_unique<SimulationCamera>(config_.width, config_.height, config_.fps);
    SimulationCamera::CameraIntrinsics intrinsics;
    intrinsics.fx = config_.fx;
    intrinsics.fy = config_.fy;
    intrinsics.cx = config_.cx;
    intrinsics.cy = config_.cy;
    src_camera_->set_intrinsics(intrinsics);

    SimulationCamera::CameraPose src_pose;
    src_pose.position = cv::Point3f(0, 0, 800);  // 俯视位置
    src_pose.rotation = cv::Point3f(0, 0, 0);
    src_pose.target_plane_distance = 0;
    src_camera_->set_pose(src_pose);

    // 观察相机（靶子前方）
    dst_camera_ = std::make_unique<SimulationCamera>(config_.width, config_.height, config_.fps);
    dst_camera_->set_intrinsics(intrinsics);

    SimulationCamera::CameraPose dst_pose;
    dst_pose.position = config_.position;
    dst_pose.rotation = cv::Point3f(camera_pitch_, camera_yaw_, 0);
    dst_pose.target_plane_distance = 0;
    dst_camera_->set_pose(dst_pose);
}

TrainingFrameGenerator::AngularVelocityFunction PentagonSimulator::energy_mechanism_velocity_generator() {
    static std::random_device rd;
    static std::mt19937 gen(rd());

    // 参数范围
    std::uniform_real_distribution<float> a_dist(0.780f, 1.045f);   // a ∈ [0.780, 1.045]
    std::uniform_real_distribution<float> omega_dist(1.884f, 2.000f); // ω ∈ [1.884, 2.000]

    // 随机生成参数
    float a = a_dist(gen);
    float omega = omega_dist(gen);
    float b = 2.090f - a;  // b = 2.090 - a

    return [a, omega, b](float t) -> float {
        return a * std::sin(omega * t) + b;
    };
}

cv::Mat PentagonSimulator::reproject_image_3d(const cv::Mat& src_image,
                                               SimulationCamera& src_camera,
                                               SimulationCamera& dst_camera) {
    int width = src_image.cols;
    int height = src_image.rows;

    auto src_intrinsics = src_camera.get_intrinsics();
    auto src_pose = src_camera.get_pose();

    // 源图像四个角的坐标
    std::vector<cv::Point2f> src_corners = {
        cv::Point2f(0, 0),
        cv::Point2f(width - 1, 0),
        cv::Point2f(width - 1, height - 1),
        cv::Point2f(0, height - 1)
    };

    // 计算这四个角在目标相机中的投影
    std::vector<cv::Point2f> dst_corners;

    for (const auto& pt : src_corners) {
        // 反投影到3D世界坐标
        float x_cam = (pt.x - src_intrinsics.cx) / src_intrinsics.fx;
        float y_cam = (pt.y - src_intrinsics.cy) / src_intrinsics.fy;

        // 靶子在 Z=0 平面
        float t = src_pose.position.z;
        cv::Point3f world_pt(x_cam * t, y_cam * t, 0.0f);

        // 投影到目标相机
        cv::Point2f proj = dst_camera.world_to_image(world_pt);
        dst_corners.push_back(proj);
    }

    // 计算透视变换矩阵
    cv::Mat M = cv::getPerspectiveTransform(src_corners, dst_corners);

    // 应用透视变换
    cv::Mat result;
    cv::warpPerspective(src_image, result, M, cv::Size(width, height),
                        cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));

    return result;
}

void PentagonSimulator::update_camera_pose() {
    SimulationCamera::CameraPose dst_pose;
    dst_pose.position = config_.position;
    dst_pose.rotation = cv::Point3f(camera_pitch_, camera_yaw_, 0);
    dst_pose.target_plane_distance = 0;
    dst_camera_->set_pose(dst_pose);
}

cv::Mat PentagonSimulator::get_frame() {
    // 获取靶子图像
    auto training_frame = generator_->get_next_frame(get_timestamp());

    last_target_src_ = training_frame.target_position;
    has_last_target_ = true;

    // 投影图像
    cv::Mat projected = reproject_image_3d(training_frame.frame, *src_camera_, *dst_camera_);

    // 计算目标色块在目标相机中的投影坐标
    auto src_intrinsics = src_camera_->get_intrinsics();
    auto src_pose = src_camera_->get_pose();
    float x_cam = (last_target_src_.x - src_intrinsics.cx) / src_intrinsics.fx;
    float y_cam = (last_target_src_.y - src_intrinsics.cy) / src_intrinsics.fy;
    float t = src_pose.position.z;
    cv::Point3f world_pt(x_cam * t, y_cam * t, 0.0f);
    last_target_dst_ = dst_camera_->world_to_image(world_pt);

    has_last_laser_ = false;
    if (config_.enable_laser) {
        const auto pose = dst_camera_->get_pose();
        const cv::Mat rotation_world_to_cam = dst_camera_->get_rotation_matrix();
        const cv::Mat rotation_cam_to_world = rotation_world_to_cam.t();

        const cv::Point3f laser_origin_local(-config_.laser_offset_left_mm, config_.laser_offset_up_mm, 0.0f);
        const cv::Point3f laser_origin_world = pose.position + mat_vec_mul(rotation_cam_to_world, laser_origin_local);

        const cv::Point3f laser_dir_world_a = normalize3(mat_vec_mul(rotation_cam_to_world, cv::Point3f(0.0f, 0.0f, 1.0f)));
        const cv::Point3f laser_dir_world_b = cv::Point3f(-laser_dir_world_a.x, -laser_dir_world_a.y, -laser_dir_world_a.z);

        cv::Point3f hit_world;
        bool hit = intersect_ray_with_plane(laser_origin_world,
                                            laser_dir_world_a,
                                            pose.target_plane_normal,
                                            pose.target_plane_distance,
                                            hit_world);
        if (!hit) {
            hit = intersect_ray_with_plane(laser_origin_world,
                                           laser_dir_world_b,
                                           pose.target_plane_normal,
                                           pose.target_plane_distance,
                                           hit_world);
        }

        if (hit) {
            const cv::Point2f hit_img = dst_camera_->world_to_image(hit_world);
            if (hit_img.x >= 0.0f && hit_img.x < static_cast<float>(projected.cols) &&
                hit_img.y >= 0.0f && hit_img.y < static_cast<float>(projected.rows)) {
                has_last_laser_ = true;
                last_laser_world_ = hit_world;
                last_laser_dst_ = hit_img;

                cv::circle(projected,
                           cv::Point(static_cast<int>(std::round(hit_img.x)), static_cast<int>(std::round(hit_img.y))),
                           static_cast<int>(std::max(3.0f, config_.laser_dot_radius_px * 2.2f)),
                           cv::Scalar(120, 120, 255),
                           -1,
                           cv::LINE_AA);
                cv::circle(projected,
                           cv::Point(static_cast<int>(std::round(hit_img.x)), static_cast<int>(std::round(hit_img.y))),
                           static_cast<int>(std::max(2.0f, config_.laser_dot_radius_px)),
                           cv::Scalar(0, 0, 255),
                           -1,
                           cv::LINE_AA);
            }
        }
    }

    // 更新帧计数
    if (!paused_) {
        frame_count_++;
    }

    return projected;
}

void PentagonSimulator::set_camera_pitch(float pitch) {
    camera_pitch_ = pitch;
    update_camera_pose();
}

void PentagonSimulator::set_camera_yaw(float yaw) {
    camera_yaw_ = yaw;
    update_camera_pose();
}

void PentagonSimulator::rotate_camera(float delta_pitch, float delta_yaw) {
    camera_pitch_ += delta_pitch;
    camera_yaw_ += delta_yaw;
    update_camera_pose();
}

void PentagonSimulator::pause() {
    paused_ = true;
    generator_->pause();
}

void PentagonSimulator::resume() {
    paused_ = false;
    generator_->resume();
}

void PentagonSimulator::reset() {
    frame_count_ = 0;
    camera_pitch_ = config_.pitch;
    camera_yaw_ = config_.yaw;
    paused_ = false;
    has_last_target_ = false;
    has_last_laser_ = false;
    update_camera_pose();
}
