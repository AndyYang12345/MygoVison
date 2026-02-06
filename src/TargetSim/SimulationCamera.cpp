#include "TargetSim/SimulationCamera.hpp"
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

SimulationCamera::SimulationCamera(int width, int height, float fps)
    : _width(width), _height(height), _fps(fps), _matrices_dirty(true) {
    // 设置默认内参数
    _intrinsics.fx = width / 2.0f;
    _intrinsics.fy = height / 2.0f;
    _intrinsics.cx = width / 2.0f;
    _intrinsics.cy = height / 2.0f;
    
    // 初始化相机矩阵
    _update_plane_transform();
}

void SimulationCamera::set_intrinsics(const CameraIntrinsics& intrinsics) {
    _intrinsics = intrinsics;
    _matrices_dirty = true;
}

void SimulationCamera::set_pose(const CameraPose& pose) {
    _pose = pose;
    _matrices_dirty = true;
}

void SimulationCamera::update_pose(const cv::Point3f& delta_position, 
                                   const cv::Point3f& delta_rotation) {
    _pose.position.x += delta_position.x;
    _pose.position.y += delta_position.y;
    _pose.position.z += delta_position.z;
    
    _pose.rotation.x += delta_rotation.x;
    _pose.rotation.y += delta_rotation.y;
    _pose.rotation.z += delta_rotation.z;
    
    _matrices_dirty = true;
}

void SimulationCamera::set_target_plane(const cv::Point3f& normal, float distance) {
    _pose.target_plane_normal = normal;
    _pose.target_plane_distance = distance;
    _matrices_dirty = true;
}

cv::Mat SimulationCamera::get_rotation_matrix() {
    if (_matrices_dirty) {
        _update_plane_transform();
    }
    return _rotation_matrix.clone();
}

cv::Mat SimulationCamera::get_camera_matrix() const {
    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = _intrinsics.fx;
    K.at<float>(1, 1) = _intrinsics.fy;
    K.at<float>(0, 2) = _intrinsics.cx;
    K.at<float>(1, 2) = _intrinsics.cy;
    return K;
}

cv::Point2f SimulationCamera::world_to_image(const cv::Point3f& world_position) {
    // 转换到相机坐标系
    cv::Mat rotation = get_rotation_matrix();
    cv::Mat world_point = cv::Mat(3, 1, CV_32F);
    world_point.at<float>(0) = world_position.x - _pose.position.x;
    world_point.at<float>(1) = world_position.y - _pose.position.y;
    world_point.at<float>(2) = world_position.z - _pose.position.z;
    
    cv::Mat camera_point = rotation * world_point;
    
    // 透视投影
    float x = camera_point.at<float>(0) / camera_point.at<float>(2);
    float y = camera_point.at<float>(1) / camera_point.at<float>(2);
    
    // 转换到图像坐标
    cv::Point2f image_pos;
    image_pos.x = _intrinsics.fx * x + _intrinsics.cx;
    image_pos.y = _intrinsics.fy * y + _intrinsics.cy;
    
    return image_pos;
}

cv::Point3f SimulationCamera::image_to_world(const cv::Point2f& image_position) {
    // 从图像坐标转换到归一化坐标
    float x_norm = (image_position.x - _intrinsics.cx) / _intrinsics.fx;
    float y_norm = (image_position.y - _intrinsics.cy) / _intrinsics.fy;
    
    // 相机坐标系中的射线方向
    cv::Point3f ray_direction(x_norm, y_norm, 1.0f);
    float ray_len = std::sqrt(ray_direction.x * ray_direction.x + 
                              ray_direction.y * ray_direction.y + 
                              ray_direction.z * ray_direction.z);
    ray_direction.x /= ray_len;
    ray_direction.y /= ray_len;
    ray_direction.z /= ray_len;
    
    // 计算射线与靶平面的交点
    // 靶平面方程：normal · (P - position) = 0
    // 射线方程：P = camera_position + t * ray_direction
    
    // 在相机坐标系中，靶平面距离相机的距离
    float t = _pose.target_plane_distance / ray_direction.z;
    
    // 相机坐标系中的点
    cv::Point3f camera_point(ray_direction.x * t, ray_direction.y * t, ray_direction.z * t);
    
    // 转换到世界坐标系
    cv::Mat rotation = get_rotation_matrix();
    cv::Mat inv_rotation = rotation.t();  // 旋转矩阵的逆等于其转置
    
    cv::Mat world_point_mat = inv_rotation * cv::Mat(3, 1, CV_32F, 
                                                      (void*)&camera_point.x);
    
    cv::Point3f world_point;
    world_point.x = world_point_mat.at<float>(0) + _pose.position.x;
    world_point.y = world_point_mat.at<float>(1) + _pose.position.y;
    world_point.z = world_point_mat.at<float>(2) + _pose.position.z;
    
    return world_point;
}

bool SimulationCamera::is_in_view(const cv::Point3f& world_position) {
    cv::Point2f image_pos = world_to_image(world_position);
    return image_pos.x >= 0 && image_pos.x < _width &&
           image_pos.y >= 0 && image_pos.y < _height;
}

std::vector<cv::Point2f> SimulationCamera::get_target_plane_projection() {
    // 获取靶平面的四个角（假设靶平面为正方形，边长为1000mm）
    float half_size = 500.0f;
    std::vector<cv::Point3f> corners = {
        cv::Point3f(-half_size, -half_size, _pose.target_plane_distance),
        cv::Point3f(half_size, -half_size, _pose.target_plane_distance),
        cv::Point3f(half_size, half_size, _pose.target_plane_distance),
        cv::Point3f(-half_size, half_size, _pose.target_plane_distance)
    };
    
    std::vector<cv::Point2f> projected;
    for (const auto& corner : corners) {
        projected.push_back(world_to_image(corner));
    }
    
    return projected;
}

cv::Mat SimulationCamera::get_perspective_matrix(const std::vector<cv::Point2f>& src_points,
                                                  const std::vector<cv::Point2f>& dst_points) {
    if (src_points.size() != 4 || dst_points.size() != 4) {
        throw std::runtime_error("需要4个源点和4个目标点进行透视变换");
    }
    
    return cv::getPerspectiveTransform(src_points, dst_points);
}

SimulationCamera::ProjectionResult SimulationCamera::project_training_frame(TrainingFrameGenerator& generator) {
    ProjectionResult result;
    
    // 获取原始训练帧
    auto training_frame = generator.get_next_frame();
    result.projected_frame = training_frame.frame.clone();
    
    // 获取靶的原始位置（像素坐标）
    cv::Point2f original_pos = training_frame.target_position;
    
    // 获取靶平面的投影四边形
    std::vector<cv::Point2f> plane_corners = get_target_plane_projection();
    
    // 检查靶是否在视野内
    result.is_in_view = is_in_view(cv::Point3f(original_pos.x - _width/2, 
                                               original_pos.y - _height/2,
                                               _pose.target_plane_distance));
    
    if (!result.is_in_view) {
        result.projected_frame = cv::Mat::zeros(_height, _width, training_frame.frame.type());
        result.distance_to_camera = 0;
        return result;
    }
    
    // 计算原始帧的四个角的世界坐标
    std::vector<cv::Point2f> src_points = {
        cv::Point2f(0, 0),
        cv::Point2f(_width, 0),
        cv::Point2f(_width, _height),
        cv::Point2f(0, _height)
    };
    
    // 应用透视变换
    cv::Mat perspective_matrix = get_perspective_matrix(src_points, plane_corners);
    cv::Mat warped_frame;
    cv::warpPerspective(training_frame.frame, warped_frame, perspective_matrix,
                       cv::Size(_width, _height));

    result.projected_frame = warped_frame;

    // 计算目标位置的透视投影坐标
    std::vector<cv::Point2f> src_pts = { original_pos };
    std::vector<cv::Point2f> dst_pts;
    cv::perspectiveTransform(src_pts, dst_pts, perspective_matrix);

    // 转换目标位置到世界坐标
    result.world_positions.push_back(image_to_world(original_pos));
    if (!dst_pts.empty()) {
        result.projected_positions.push_back(dst_pts[0]);
    }
    
    // 计算距离
    cv::Point3f target_3d = result.world_positions[0];
    result.distance_to_camera = std::sqrt(
        (target_3d.x - _pose.position.x) * (target_3d.x - _pose.position.x) +
        (target_3d.y - _pose.position.y) * (target_3d.y - _pose.position.y) +
        (target_3d.z - _pose.position.z) * (target_3d.z - _pose.position.z)
    );
    
    return result;
}

SimulationCamera::ProjectionResult SimulationCamera::project_target_position(const cv::Point2f& target_position,
                                                                              const cv::Mat& original_frame) {
    ProjectionResult result;
    result.projected_frame = original_frame.clone();
    
    // 转换目标位置到世界坐标
    cv::Point3f world_pos = image_to_world(target_position);
    result.world_positions.push_back(world_pos);
    result.projected_positions.push_back(target_position);
    
    // 检查是否在视野内
    result.is_in_view = is_in_view(world_pos);
    
    // 计算距离
    result.distance_to_camera = std::sqrt(
        (world_pos.x - _pose.position.x) * (world_pos.x - _pose.position.x) +
        (world_pos.y - _pose.position.y) * (world_pos.y - _pose.position.y) +
        (world_pos.z - _pose.position.z) * (world_pos.z - _pose.position.z)
    );
    
    return result;
}

cv::Mat SimulationCamera::apply_motion_blur(const cv::Mat& frame, const cv::Point2f& velocity, int blur_strength) {
    if (blur_strength <= 0) return frame.clone();
    
    blur_strength = std::min(blur_strength, 5);
    int kernel_size = 2 * blur_strength + 1;
    
    // 创建运动模糊核（手动实现）
    float angle = std::atan2(velocity.y, velocity.x);
    
    // 手动创建直线形状的运动模糊核
    cv::Mat kernel = cv::Mat::zeros(kernel_size, kernel_size, CV_32F);
    
    int cx = kernel_size / 2;
    int cy = kernel_size / 2;
    
    float cos_a = std::cos(angle);
    float sin_a = std::sin(angle);
    
    // 在核中绘制直线
    for (int i = -kernel_size/2; i <= kernel_size/2; i++) {
        int x = static_cast<int>(i * cos_a);
        int y = static_cast<int>(i * sin_a);
        if (x + cx >= 0 && x + cx < kernel_size && y + cy >= 0 && y + cy < kernel_size) {
            kernel.at<float>(y + cy, x + cx) = 1.0f;
        }
    }
    
    // 归一化核
    kernel = kernel / cv::sum(kernel)[0];
    
    cv::Mat blurred;
    cv::filter2D(frame, blurred, -1, kernel);
    
    return blurred;
}

cv::Mat SimulationCamera::apply_focus_effect(const cv::Mat& frame, float focus_distance, float aperture) {
    // 这是一个简化的焦点效果实现
    // 通过高斯模糊模拟景深效果
    
    float blur_amount = std::abs(focus_distance - _pose.target_plane_distance) / 100.0f * aperture;
    blur_amount = std::min(blur_amount, 10.0f);
    
    if (blur_amount < 0.5f) return frame.clone();
    
    int kernel_size = static_cast<int>(blur_amount * 2) * 2 + 1;
    
    cv::Mat focused;
    cv::GaussianBlur(frame, focused, cv::Size(kernel_size, kernel_size), blur_amount);
    
    return focused;
}

cv::Mat SimulationCamera::apply_lens_distortion(const cv::Mat& frame) {
    if (_intrinsics.k1 == 0 && _intrinsics.k2 == 0 && 
        _intrinsics.p1 == 0 && _intrinsics.p2 == 0) {
        return frame.clone();
    }
    
    // 简化版本：使用桶形畸变近似
    // 实际项目中可以使用OpenCV的undistort函数
    
    cv::Mat distorted = frame.clone();
    
    // 计算畸变程度
    float distortion = std::sqrt(_intrinsics.k1 * _intrinsics.k1 + _intrinsics.k2 * _intrinsics.k2);
    
    if (distortion > 0.01f) {
        // 应用简单的缩放变换来模拟畸变
        float scale = 1.0f + distortion * 0.1f;
        cv::Mat transform_matrix = cv::getRotationMatrix2D(
            cv::Point2f(_width / 2.0f, _height / 2.0f),
            0,
            scale
        );
        cv::warpAffine(frame, distorted, transform_matrix, frame.size());
    }
    
    return distorted;
}

cv::Point3f SimulationCamera::_euler_to_rotation_vector(const cv::Point3f& euler) {
    // 简化的欧拉角到旋转向量的转换
    // 这里使用ZYX约定
    return euler;
}

cv::Point3f SimulationCamera::_rotation_vector_to_euler(const cv::Point3f& rot_vec) {
    return rot_vec;
}

void SimulationCamera::_update_plane_transform() {
    // 计算旋转矩阵
    // 使用cv::Rodrigues从旋转向量计算旋转矩阵
    
    float pitch = _pose.rotation.x;
    float yaw = _pose.rotation.y;
    float roll = _pose.rotation.z;
    
    // 计算旋转矩阵（ZYX约定）
    float cy = std::cos(yaw);
    float sy = std::sin(yaw);
    float cp = std::cos(pitch);
    float sp = std::sin(pitch);
    float cr = std::cos(roll);
    float sr = std::sin(roll);
    
    _rotation_matrix = cv::Mat(3, 3, CV_32F);
    
    _rotation_matrix.at<float>(0, 0) = cy * cr + sy * sp * sr;
    _rotation_matrix.at<float>(0, 1) = -cy * sr + sy * sp * cr;
    _rotation_matrix.at<float>(0, 2) = sy * cp;
    
    _rotation_matrix.at<float>(1, 0) = cp * sr;
    _rotation_matrix.at<float>(1, 1) = cp * cr;
    _rotation_matrix.at<float>(1, 2) = -sp;
    
    _rotation_matrix.at<float>(2, 0) = -sy * cr + cy * sp * sr;
    _rotation_matrix.at<float>(2, 1) = sy * sr + cy * sp * cr;
    _rotation_matrix.at<float>(2, 2) = cy * cp;
    
    // 计算相机矩阵（已在get_camera_matrix中实现）
    _matrices_dirty = false;
}
