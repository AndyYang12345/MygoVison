#include "TargetSim/Target3DGenerator.hpp"

#include <algorithm>
#include <cmath>

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

Target3DGenerator::Target3DGenerator(int width, int height, float fps)
    : _generator(width, height, fps),
      _camera(width, height, fps),
      _random_interval(0.6f),
      _laser_enabled(true),
    _laser_offset_left_mm(0.0f),
    _laser_offset_up_mm(20.0f),
      _laser_dot_radius_px(4.0f),
      _has_last_laser(false),
      _last_laser_projected(-1.0f, -1.0f) {
    _generator.set_training_mode(TrainingFrameGenerator::MODE_RANDOM_APPEARANCE, _random_interval);
}

void Target3DGenerator::set_random_interval(float seconds) {
    _random_interval = seconds;
    _generator.set_training_mode(TrainingFrameGenerator::MODE_RANDOM_APPEARANCE, _random_interval);
}

void Target3DGenerator::set_camera_intrinsics(const SimulationCamera::CameraIntrinsics& intrinsics) {
    _camera.set_intrinsics(intrinsics);
}

void Target3DGenerator::set_camera_pose(const SimulationCamera::CameraPose& pose) {
    _camera.set_pose(pose);
}

void Target3DGenerator::set_target_plane(const cv::Point3f& plane_normal, float distance) {
    _camera.set_target_plane(plane_normal, distance);
}

void Target3DGenerator::set_laser_model(bool enabled,
                                        float offset_left_mm,
                                        float dot_radius_px,
                                        float offset_up_mm) {
    _laser_enabled = enabled;
    _laser_offset_left_mm = offset_left_mm;
    _laser_offset_up_mm = offset_up_mm;
    _laser_dot_radius_px = dot_radius_px;
}

SimulationCamera::ProjectionResult Target3DGenerator::generate_projected_frame() {
    auto result = _camera.project_training_frame(_generator);

    _has_last_laser = false;
    _last_laser_projected = cv::Point2f(-1.0f, -1.0f);

    if (!_laser_enabled || result.projected_frame.empty()) {
        return result;
    }

    const auto pose = _camera.get_pose();
    const cv::Mat rotation_world_to_cam = _camera.get_rotation_matrix();
    const cv::Mat rotation_cam_to_world = rotation_world_to_cam.t();

    const cv::Point3f laser_origin_local(-_laser_offset_left_mm, _laser_offset_up_mm, 0.0f);
    const cv::Point3f origin_world = pose.position + mat_vec_mul(rotation_cam_to_world, laser_origin_local);
    const cv::Point3f dir_world_a = normalize3(mat_vec_mul(rotation_cam_to_world, cv::Point3f(0.0f, 0.0f, 1.0f)));
    const cv::Point3f dir_world_b(-dir_world_a.x, -dir_world_a.y, -dir_world_a.z);

    cv::Point3f hit_world;
    bool hit = intersect_ray_with_plane(origin_world,
                                        dir_world_a,
                                        pose.target_plane_normal,
                                        pose.target_plane_distance,
                                        hit_world);
    if (!hit) {
        hit = intersect_ray_with_plane(origin_world,
                                       dir_world_b,
                                       pose.target_plane_normal,
                                       pose.target_plane_distance,
                                       hit_world);
    }
    if (!hit) {
        return result;
    }

    const cv::Point2f hit_img = _camera.world_to_image(hit_world);
    if (hit_img.x < 0.0f || hit_img.x >= static_cast<float>(result.projected_frame.cols) ||
        hit_img.y < 0.0f || hit_img.y >= static_cast<float>(result.projected_frame.rows)) {
        return result;
    }

    cv::circle(result.projected_frame,
               cv::Point(static_cast<int>(std::round(hit_img.x)), static_cast<int>(std::round(hit_img.y))),
               static_cast<int>(std::max(3.0f, _laser_dot_radius_px * 2.2f)),
               cv::Scalar(120, 120, 255),
               -1,
               cv::LINE_AA);
    cv::circle(result.projected_frame,
               cv::Point(static_cast<int>(std::round(hit_img.x)), static_cast<int>(std::round(hit_img.y))),
               static_cast<int>(std::max(2.0f, _laser_dot_radius_px)),
               cv::Scalar(0, 0, 255),
               -1,
               cv::LINE_AA);

    _has_last_laser = true;
    _last_laser_projected = hit_img;
    result.projected_positions.push_back(hit_img);
    result.world_positions.push_back(hit_world);
    return result;
}
